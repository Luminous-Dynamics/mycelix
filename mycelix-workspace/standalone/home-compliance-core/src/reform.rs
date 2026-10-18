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

/// Immutable summary of an actually completed public consultation. Readiness
/// to *open* consultation is not proof that consultation happened. The archive
/// adapter must later verify notice, feedback, and response bytes independently.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PublicConsultationCloseout {
    pub consultation_id: String,
    pub dossier_id: String,
    pub dossier_snapshot_sha256: String,
    /// Digest of the canonical closeout record, excluding this self-digest field.
    pub closeout_snapshot_sha256: String,
    pub jurisdiction_path: String,
    pub opened_on_day: u32,
    pub closed_on_day: u32,
    pub public_notice_uri: String,
    pub feedback_archive_uri: String,
    pub feedback_archive_sha256: String,
    pub feedback_count: u64,
    /// Explain how comments were considered, including when no comments arrived.
    pub response_summary: String,
    pub response_record_uri: String,
    pub response_record_sha256: String,
}

/// Formal decision disposition recorded by a legally empowered body.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdoptionDisposition {
    Adopted,
    Rejected,
    ReturnedForRevision,
    Deferred,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AdoptionDecisionRecord {
    pub decision_id: String,
    pub dossier_id: String,
    pub dossier_snapshot_sha256: String,
    pub consultation_id: String,
    /// Pins the exact consultation closeout snapshot used for the decision.
    pub consultation_closeout_sha256: String,
    pub jurisdiction_path: String,
    pub authority_id: String,
    pub decision_maker_id: String,
    /// Exact legal/administrative instrument empowering the decision maker.
    pub authority_basis_uri: String,
    /// Publicly discoverable decision or adopting instrument.
    pub public_decision_uri: String,
    pub public_decision_sha256: String,
    pub disposition: AdoptionDisposition,
    pub decided_on_day: u32,
    /// Exclusive legal effect is jurisdiction-specific; this field only records
    /// the source-asserted effective date and never establishes current law.
    pub effective_from_day: Option<u32>,
}

/// Errors in the transition from proposal to public consultation and legal
/// adoption. These are record-consistency errors, not legal-authenticity proofs.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ReformWorkflowIssue {
    DossierNotReadyForConsultation,
    EmptyField(&'static str),
    InvalidJurisdictionPath(&'static str),
    InvalidDigest(&'static str),
    DossierIdMismatch,
    DossierSnapshotMismatch,
    ConsultationIdMismatch,
    ConsultationCloseoutSnapshotMismatch,
    JurisdictionMismatch,
    InvalidConsultationDates,
    AdoptionPredatesConsultationClose,
    AdoptedWithoutEffectiveDate,
    EffectiveDateBeforeDecision,
    NonAdoptionHasEffectiveDate,
}

/// Opaque result of checking cross-record consistency and required metadata.
/// This does NOT authenticate an authority, signature, source bytes, or legal
/// effect; those require a trusted jurisdictional registry adapter.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RecordedAdoptionMetadata {
    record: AdoptionDecisionRecord,
}

impl RecordedAdoptionMetadata {
    pub fn decision_id(&self) -> &str {
        &self.record.decision_id
    }

    pub fn disposition(&self) -> AdoptionDisposition {
        self.record.disposition
    }

    pub fn jurisdiction_path(&self) -> &str {
        &self.record.jurisdiction_path
    }

    pub fn effective_from_day(&self) -> Option<u32> {
        self.record.effective_from_day
    }

    pub fn public_decision_sha256(&self) -> &str {
        &self.record.public_decision_sha256
    }
}

fn validate_closeout(
    dossier: &CodeReformDossier,
    closeout: &PublicConsultationCloseout,
) -> Result<(), Vec<ReformWorkflowIssue>> {
    let mut issues = Vec::new();
    if evaluate_reform_dossier(dossier).status != ReformDossierStatus::ReadyForPublicConsultation {
        issues.push(ReformWorkflowIssue::DossierNotReadyForConsultation);
    }
    if closeout.dossier_id != dossier.dossier_id {
        issues.push(ReformWorkflowIssue::DossierIdMismatch);
    }
    if closeout.dossier_snapshot_sha256 != dossier.snapshot_sha256 {
        issues.push(ReformWorkflowIssue::DossierSnapshotMismatch);
    }
    if closeout.jurisdiction_path != dossier.jurisdiction_path {
        issues.push(ReformWorkflowIssue::JurisdictionMismatch);
    }
    if closeout.closed_on_day < closeout.opened_on_day {
        issues.push(ReformWorkflowIssue::InvalidConsultationDates);
    }
    if !valid_jurisdiction_path(&closeout.jurisdiction_path) {
        issues.push(ReformWorkflowIssue::InvalidJurisdictionPath("consultation"));
    }
    for (field, value) in [
        ("consultation_id", closeout.consultation_id.as_str()),
        ("public_notice_uri", closeout.public_notice_uri.as_str()),
        ("feedback_archive_uri", closeout.feedback_archive_uri.as_str()),
        ("response_summary", closeout.response_summary.as_str()),
        ("response_record_uri", closeout.response_record_uri.as_str()),
    ] {
        if !nonblank(value) {
            issues.push(ReformWorkflowIssue::EmptyField(field));
        }
    }
    if !is_sha256_hex(&closeout.dossier_snapshot_sha256) {
        issues.push(ReformWorkflowIssue::InvalidDigest("dossier_snapshot_sha256"));
    }
    if !is_sha256_hex(&closeout.closeout_snapshot_sha256) {
        issues.push(ReformWorkflowIssue::InvalidDigest("closeout_snapshot_sha256"));
    }
    if !is_sha256_hex(&closeout.feedback_archive_sha256) {
        issues.push(ReformWorkflowIssue::InvalidDigest("feedback_archive_sha256"));
    }
    if !is_sha256_hex(&closeout.response_record_sha256) {
        issues.push(ReformWorkflowIssue::InvalidDigest("response_record_sha256"));
    }

    if issues.is_empty() {
        Ok(())
    } else {
        Err(issues)
    }
}

/// Validate a public-consultation closeout against the immutable dossier it
/// claims to close. A count of zero is allowed only with an explicit response
/// record; the system never invents evidence that comments were received.
pub fn validate_public_consultation_closeout(
    dossier: &CodeReformDossier,
    closeout: &PublicConsultationCloseout,
) -> Result<(), Vec<ReformWorkflowIssue>> {
    validate_closeout(dossier, closeout)
}

/// Validate an authority's adoption/rejection decision after consultation.
/// This function checks typed metadata, jurisdiction, dates, and snapshot links.
/// It cannot prove that the authority or its signatory is legitimate, that the
/// referenced law authorizes this decision, or that the linked bytes are authentic.
pub fn validate_adoption_decision(
    dossier: &CodeReformDossier,
    closeout: &PublicConsultationCloseout,
    decision: AdoptionDecisionRecord,
) -> Result<RecordedAdoptionMetadata, Vec<ReformWorkflowIssue>> {
    let mut issues = validate_closeout(dossier, closeout).err().unwrap_or_default();

    if decision.dossier_id != dossier.dossier_id {
        issues.push(ReformWorkflowIssue::DossierIdMismatch);
    }
    if decision.dossier_snapshot_sha256 != dossier.snapshot_sha256 {
        issues.push(ReformWorkflowIssue::DossierSnapshotMismatch);
    }
    if decision.consultation_id != closeout.consultation_id {
        issues.push(ReformWorkflowIssue::ConsultationIdMismatch);
    }
    if decision.consultation_closeout_sha256 != closeout.closeout_snapshot_sha256 {
        issues.push(ReformWorkflowIssue::ConsultationCloseoutSnapshotMismatch);
    }
    if !is_sha256_hex(&decision.consultation_closeout_sha256) {
        issues.push(ReformWorkflowIssue::InvalidDigest("consultation_closeout_sha256"));
    }
    if decision.jurisdiction_path != dossier.jurisdiction_path {
        issues.push(ReformWorkflowIssue::JurisdictionMismatch);
    }
    if decision.decided_on_day < closeout.closed_on_day {
        issues.push(ReformWorkflowIssue::AdoptionPredatesConsultationClose);
    }
    if !valid_jurisdiction_path(&decision.jurisdiction_path) {
        issues.push(ReformWorkflowIssue::InvalidJurisdictionPath("decision"));
    }
    for (field, value) in [
        ("decision_id", decision.decision_id.as_str()),
        ("authority_id", decision.authority_id.as_str()),
        ("decision_maker_id", decision.decision_maker_id.as_str()),
        ("authority_basis_uri", decision.authority_basis_uri.as_str()),
        ("public_decision_uri", decision.public_decision_uri.as_str()),
    ] {
        if !nonblank(value) {
            issues.push(ReformWorkflowIssue::EmptyField(field));
        }
    }
    if !is_sha256_hex(&decision.dossier_snapshot_sha256) {
        issues.push(ReformWorkflowIssue::InvalidDigest("decision dossier_snapshot_sha256"));
    }
    if !is_sha256_hex(&decision.public_decision_sha256) {
        issues.push(ReformWorkflowIssue::InvalidDigest("public_decision_sha256"));
    }
    match (decision.disposition, decision.effective_from_day) {
        (AdoptionDisposition::Adopted, None) => {
            issues.push(ReformWorkflowIssue::AdoptedWithoutEffectiveDate);
        }
        (AdoptionDisposition::Adopted, Some(day)) if day < decision.decided_on_day => {
            issues.push(ReformWorkflowIssue::EffectiveDateBeforeDecision);
        }
        (AdoptionDisposition::Rejected, Some(_))
        | (AdoptionDisposition::ReturnedForRevision, Some(_))
        | (AdoptionDisposition::Deferred, Some(_)) => {
            issues.push(ReformWorkflowIssue::NonAdoptionHasEffectiveDate);
        }
        _ => {}
    }

    if issues.is_empty() {
        Ok(RecordedAdoptionMetadata { record: decision })
    } else {
        Err(issues)
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

    fn consultation_ready_dossier() -> CodeReformDossier {
        let mut current = dossier();
        current.independent_reviews = vec![
            review("reviewer:a", IndependentReviewDisposition::SupportsPublicConsultation),
            review("reviewer:b", IndependentReviewDisposition::ChallengesProposal),
        ];
        current
    }

    fn consultation_closeout(dossier: &CodeReformDossier) -> PublicConsultationCloseout {
        PublicConsultationCloseout {
            consultation_id: "consultation:ventilation:2026".into(),
            dossier_id: dossier.dossier_id.clone(),
            dossier_snapshot_sha256: dossier.snapshot_sha256.clone(),
            closeout_snapshot_sha256: "9".repeat(64),
            jurisdiction_path: dossier.jurisdiction_path.clone(),
            opened_on_day: 110,
            closed_on_day: 120,
            public_notice_uri: "https://example.invalid/consultation/notice".into(),
            feedback_archive_uri: "https://example.invalid/consultation/feedback".into(),
            feedback_archive_sha256: "d".repeat(64),
            feedback_count: 17,
            response_summary: "All comments were categorized and answered; dissent is preserved".into(),
            response_record_uri: "https://example.invalid/consultation/responses".into(),
            response_record_sha256: "e".repeat(64),
        }
    }

    fn adoption_decision(
        dossier: &CodeReformDossier,
        closeout: &PublicConsultationCloseout,
    ) -> AdoptionDecisionRecord {
        AdoptionDecisionRecord {
            decision_id: "decision:building-board:2026-01".into(),
            dossier_id: dossier.dossier_id.clone(),
            dossier_snapshot_sha256: dossier.snapshot_sha256.clone(),
            consultation_id: closeout.consultation_id.clone(),
            consultation_closeout_sha256: closeout.closeout_snapshot_sha256.clone(),
            jurisdiction_path: dossier.jurisdiction_path.clone(),
            authority_id: "authority:lawfully-designated-building-board".into(),
            decision_maker_id: "decision-maker:recorded-signatory".into(),
            authority_basis_uri: "https://example.invalid/legal-authority-basis".into(),
            public_decision_uri: "https://example.invalid/official-decision".into(),
            public_decision_sha256: "f".repeat(64),
            disposition: AdoptionDisposition::Adopted,
            decided_on_day: 125,
            effective_from_day: Some(140),
        }
    }

    #[test]
    fn adoption_metadata_requires_consultation_closeout_and_preserves_dissent_snapshot() {
        let dossier = consultation_ready_dossier();
        assert_eq!(
            evaluate_reform_dossier(&dossier).status,
            ReformDossierStatus::ReadyForPublicConsultation
        );
        let closeout = consultation_closeout(&dossier);
        assert_eq!(validate_public_consultation_closeout(&dossier, &closeout), Ok(()));
        let recorded = validate_adoption_decision(
            &dossier,
            &closeout,
            adoption_decision(&dossier, &closeout),
        )
        .expect("consistent adoption metadata should validate");
        assert_eq!(recorded.disposition(), AdoptionDisposition::Adopted);
        assert_eq!(recorded.effective_from_day(), Some(140));
        assert_eq!(recorded.public_decision_sha256(), "f".repeat(64));
        assert_eq!(
            dossier.independent_reviews[1].disposition,
            IndependentReviewDisposition::ChallengesProposal
        );
    }

    #[test]
    fn adoption_cannot_precede_closed_public_consultation() {
        let dossier = consultation_ready_dossier();
        let mut closeout = consultation_closeout(&dossier);
        closeout.opened_on_day = 120;
        closeout.closed_on_day = 110;
        assert!(validate_public_consultation_closeout(&dossier, &closeout)
            .unwrap_err()
            .contains(&ReformWorkflowIssue::InvalidConsultationDates));
        let mut decision = adoption_decision(&dossier, &closeout);
        decision.decided_on_day = 109;
        assert!(validate_adoption_decision(&dossier, &closeout, decision)
            .unwrap_err()
            .contains(&ReformWorkflowIssue::InvalidConsultationDates));
    }

    #[test]
    fn adoption_must_pin_the_consulted_dossier_snapshot() {
        let dossier = consultation_ready_dossier();
        let closeout = consultation_closeout(&dossier);
        let mut decision = adoption_decision(&dossier, &closeout);
        decision.dossier_snapshot_sha256 = "1".repeat(64);
        assert!(validate_adoption_decision(&dossier, &closeout, decision)
            .unwrap_err()
            .contains(&ReformWorkflowIssue::DossierSnapshotMismatch));
    }

    #[test]
    fn adoption_record_pins_the_exact_consultation_closeout_snapshot() {
        let dossier = consultation_ready_dossier();
        let closeout = consultation_closeout(&dossier);
        let mut decision = adoption_decision(&dossier, &closeout);
        decision.consultation_closeout_sha256 = "8".repeat(64);
        assert!(validate_adoption_decision(&dossier, &closeout, decision)
            .unwrap_err()
            .contains(&ReformWorkflowIssue::ConsultationCloseoutSnapshotMismatch));
    }

    #[test]
    fn only_adopted_dispositions_may_have_a_future_effective_date() {
        let dossier = consultation_ready_dossier();
        let closeout = consultation_closeout(&dossier);
        let mut adopted_without_date = adoption_decision(&dossier, &closeout);
        adopted_without_date.effective_from_day = None;
        assert!(validate_adoption_decision(&dossier, &closeout, adopted_without_date)
            .unwrap_err()
            .contains(&ReformWorkflowIssue::AdoptedWithoutEffectiveDate));

        let mut rejected_with_date = adoption_decision(&dossier, &closeout);
        rejected_with_date.disposition = AdoptionDisposition::Rejected;
        assert!(validate_adoption_decision(&dossier, &closeout, rejected_with_date)
            .unwrap_err()
            .contains(&ReformWorkflowIssue::NonAdoptionHasEffectiveDate));
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
