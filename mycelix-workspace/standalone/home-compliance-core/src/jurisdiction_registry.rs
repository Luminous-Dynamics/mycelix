//! Auditable capability cards for global jurisdiction rollout.
//!
//! A jurisdiction is not labelled supported merely because its name exists in
//! the registry or a global model code is imported. The card records its bounded
//! scope, coverage by regulatory domain, source-registry snapshot, adversarial
//! test corpus, update policy, accountable maintainer, and independent reviews.
//! This module validates card metadata; it does not authenticate identities,
//! verify source bytes, interpret laws, or certify a jurisdiction as legally
//! compliant. Those remain responsibilities of trusted adapters and authorities.

use std::collections::HashSet;

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

/// Regulatory domains a jurisdiction profile must explicitly account for.
/// An area may be marked not applicable only with a reviewed rationale and
/// decision reference; silently omitting a domain is never equivalent to N/A.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum RegulatoryDomain {
    ResidentialBuilding,
    PlanningAndLandUse,
    StructuralSafety,
    FireAndLifeSafety,
    EnergyAndThermal,
    Accessibility,
    WaterAndSanitation,
    ElectricalInstallations,
    PermitAndInspectionProcess,
    HeritageAndConservation,
    EnvironmentalAndSiteHazards,
}

pub const REQUIRED_REGULATORY_DOMAINS: [RegulatoryDomain; 11] = [
    RegulatoryDomain::ResidentialBuilding,
    RegulatoryDomain::PlanningAndLandUse,
    RegulatoryDomain::StructuralSafety,
    RegulatoryDomain::FireAndLifeSafety,
    RegulatoryDomain::EnergyAndThermal,
    RegulatoryDomain::Accessibility,
    RegulatoryDomain::WaterAndSanitation,
    RegulatoryDomain::ElectricalInstallations,
    RegulatoryDomain::PermitAndInspectionProcess,
    RegulatoryDomain::HeritageAndConservation,
    RegulatoryDomain::EnvironmentalAndSiteHazards,
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum JurisdictionSupportStatus {
    Draft,
    Researching,
    Pilot,
    Supported,
    Suspended,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DomainCoverageDisposition {
    Covered {
        module_ids: Vec<String>,
        source_snapshot_sha256: String,
    },
    ExplicitlyNotApplicable {
        rationale: String,
        decision_reference: String,
    },
    Missing {
        gap_description: String,
    },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DomainCoverageRecord {
    pub domain: RegulatoryDomain,
    pub disposition: DomainCoverageDisposition,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CapabilityReviewDisposition {
    SupportsBoundedSupportClaim,
    ChallengesSupportClaim,
    Inconclusive,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilityReview {
    pub reviewer_id: String,
    pub review_scope: String,
    /// Hash of the canonical card subject excluding the review-attestation list,
    /// avoiding a self-referential digest.
    pub reviewed_subject_sha256: String,
    pub disposition: CapabilityReviewDisposition,
    pub rationale: String,
    pub conflict_of_interest_disclosure: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JurisdictionCapabilityCard {
    /// Canonical path, e.g. ZA/GP/Johannesburg. It is an identifier, not proof
    /// that the jurisdiction's geographic or legal hierarchy has been verified.
    pub jurisdiction_id: String,
    pub display_name: String,
    pub declared_status: JurisdictionSupportStatus,
    /// Identity accountable for authoring the card subject, distinct from the
    /// ongoing maintainer and every independent reviewer.
    pub card_author_id: String,
    /// Digest of canonical card content excluding reviewer attestations.
    pub subject_sha256: String,
    pub source_registry_snapshot_sha256: String,
    pub test_corpus_sha256: String,
    pub tested_case_count: u32,
    pub coverage: Vec<DomainCoverageRecord>,
    pub independent_reviews: Vec<CapabilityReview>,
    pub known_scope_gaps: Vec<String>,
    /// Human-readable + machine-actionable policy for monitoring and updating
    /// the jurisdiction's sources, tests, and supported scope.
    pub update_policy: String,
    pub accountable_maintainer: String,
    pub source_registry_checked_on_day: u32,
    pub registry_as_of_day: u32,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CapabilityCardIssue {
    EmptyField(&'static str),
    InvalidJurisdictionId,
    InvalidSubjectDigest,
    InvalidSourceRegistryDigest,
    InvalidTestCorpusDigest,
    NoTestCases,
    StaleSourceRegistry,
    FutureSourceRegistryCheck,
    EmptyCoverage,
    DuplicateCoverageDomain(RegulatoryDomain),
    MissingRequiredDomain(RegulatoryDomain),
    InvalidCoverageModuleId {
        domain: RegulatoryDomain,
    }
    InvalidCoverageSourceDigest {
        domain: RegulatoryDomain,
    }
    InvalidNotApplicableRationale {
        domain: RegulatoryDomain,
    }
    InvalidNotApplicableDecision {
        domain: RegulatoryDomain,
    }
    InvalidMissingGapDescription {
        domain: RegulatoryDomain,
    }
    DuplicateReview(String),
    AuthorIncludedAsReviewer(String),
    InvalidReviewMetadata {
        reviewer_id: String,
        field: &'static str,
    }
    InvalidReviewDigest(String),
    ReviewDigestMismatch(String),
    InsufficientIndependentReviewers,
    NoIndependentReviewerSupportsClaim,
    IndependentReviewerChallengesClaim(String),
    IndependentReviewerInconclusive(String),
    KnownGapUndisclosedOrBlank,
}

/// Results are metadata-validation states, not legal approval states.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CapabilityCardValidationStatus {
    MetadataIncomplete,
    ResearchOnly,
    PilotMetadataComplete,
    SupportedClaimMetadataComplete,
    Suspended,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilityCardReport {
    pub status: CapabilityCardValidationStatus,
    pub issues: Vec<CapabilityCardIssue>,
    /// True only when the declared Supported claim has complete required
    /// metadata. This does not authenticate sources or prove legal correctness.
    pub supported_claim_metadata_complete: bool,
}

/// Check whether a claimed support profile has enough documented coverage and
/// governance metadata to be published as a candidate supported profile.
///
/// Freshness is an explicit caller policy (not a universal legal threshold).
/// Source hashes and reviewer IDs are syntax/consistency checks only. Production
/// must authenticate the card, signed reviews, source artifacts, and authority.
pub fn validate_capability_card(
    card: &JurisdictionCapabilityCard,
    maximum_source_age_days: u32,
) -> CapabilityCardReport {
    let mut issues = Vec::new();
    for (field, value) in [
        ("jurisdiction_id", card.jurisdiction_id.as_str()),
        ("display_name", card.display_name.as_str()),
        ("card_author_id", card.card_author_id.as_str()),
        ("update_policy", card.update_policy.as_str()),
        (
            "accountable_maintainer",
            card.accountable_maintainer.as_str(),
        ),
    ] {
        if !nonblank(value) {
            issues.push(CapabilityCardIssue::EmptyField(field));
        }
    }
    if !valid_jurisdiction_path(&card.jurisdiction_id) {
        issues.push(CapabilityCardIssue::InvalidJurisdictionId);
    }
    if !is_sha256_hex(&card.subject_sha256) {
        issues.push(CapabilityCardIssue::InvalidSubjectDigest);
    }
    if !is_sha256_hex(&card.source_registry_snapshot_sha256) {
        issues.push(CapabilityCardIssue::InvalidSourceRegistryDigest);
    }
    if !is_sha256_hex(&card.test_corpus_sha256) {
        issues.push(CapabilityCardIssue::InvalidTestCorpusDigest);
    }
    if card.tested_case_count == 0 {
        issues.push(CapabilityCardIssue::NoTestCases);
    }
    for gap in &card.known_scope_gaps {
        if !nonblank(gap) {
            issues.push(CapabilityCardIssue::KnownGapUndisclosedOrBlank);
        }
    }
    if card.source_registry_checked_on_day > card.registry_as_of_day {
        issues.push(CapabilityCardIssue::FutureSourceRegistryCheck);
    } else if card
        .registry_as_of_day
        .saturating_sub(card.source_registry_checked_on_day)
        > maximum_source_age_days
    {
        issues.push(CapabilityCardIssue::StaleSourceRegistry);
    }

    if card.coverage.is_empty() {
        issues.push(CapabilityCardIssue::EmptyCoverage);
    }
    let mut domains = HashSet::new();
    for record in &card.coverage {
        if !domains.insert(record.domain) {
            issues.push(CapabilityCardIssue::DuplicateCoverageDomain(record.domain));
            continue;
        }
        match &record.disposition {
            DomainCoverageDisposition::Covered {
                module_ids,
                source_snapshot_sha256,
            } => {
                if module_ids.is_empty() || module_ids.iter().any(|id| !nonblank(id)) {
                    issues.push(CapabilityCardIssue::InvalidCoverageModuleId {
                        domain: record.domain,
                    });
                }
                let mut unique = HashSet::new();
                if module_ids.iter().any(|id| !unique.insert(id.as_str())) {
                    issues.push(CapabilityCardIssue::InvalidCoverageModuleId {
                        domain: record.domain,
                    });
                }
                if !is_sha256_hex(source_snapshot_sha256) {
                    issues.push(CapabilityCardIssue::InvalidCoverageSourceDigest {
                        domain: record.domain,
                    });
                }
            }
            DomainCoverageDisposition::ExplicitlyNotApplicable {
                rationale,
                decision_reference,
            } => {
                if !nonblank(rationale) {
                    issues.push(CapabilityCardIssue::InvalidNotApplicableRationale {
                        domain: record.domain,
                    });
                }
                if !nonblank(decision_reference) {
                    issues.push(CapabilityCardIssue::InvalidNotApplicableDecision {
                        domain: record.domain,
                    });
                }
            }
            DomainCoverageDisposition::Missing { gap_description } => {
                if !nonblank(gap_description) {
                    issues.push(CapabilityCardIssue::InvalidMissingGapDescription {
                        domain: record.domain,
                    });
                }
                if card.declared_status == JurisdictionSupportStatus::Supported {
                    issues.push(CapabilityCardIssue::MissingRequiredDomain(record.domain));
                }
            }
        }
    }
    for domain in REQUIRED_REGULATORY_DOMAINS {
        if !domains.contains(&domain)
            && card.declared_status == JurisdictionSupportStatus::Supported
        {
            issues.push(CapabilityCardIssue::MissingRequiredDomain(domain));
        }
    }

    let mut reviewer_ids = HashSet::new();
    for review in &card.independent_reviews {
        if !nonblank(&review.reviewer_id) {
            issues.push(CapabilityCardIssue::InvalidReviewMetadata {
                reviewer_id: review.reviewer_id.clone(),
                field: "reviewer_id",
            });
        }
        if !reviewer_ids.insert(review.reviewer_id.as_str()) {
            issues.push(CapabilityCardIssue::DuplicateReview(
                review.reviewer_id.clone(),
            ));
        }
        if review.reviewer_id == card.card_author_id
            || review.reviewer_id == card.accountable_maintainer
        {
            issues.push(CapabilityCardIssue::AuthorIncludedAsReviewer(
                review.reviewer_id.clone(),
            ));
        }
        for (field, value) in [
            ("review_scope", review.review_scope.as_str()),
            ("rationale", review.rationale.as_str()),
            (
                "conflict_of_interest_disclosure",
                review.conflict_of_interest_disclosure.as_str(),
            ),
        ] {
            if !nonblank(value) {
                issues.push(CapabilityCardIssue::InvalidReviewMetadata {
                    reviewer_id: review.reviewer_id.clone(),
                    field,
                });
            }
        }
        if !is_sha256_hex(&review.reviewed_subject_sha256) {
            issues.push(CapabilityCardIssue::InvalidReviewDigest(
                review.reviewer_id.clone(),
            ));
        } else if review.reviewed_subject_sha256 != card.subject_sha256 {
            issues.push(CapabilityCardIssue::ReviewDigestMismatch(
                review.reviewer_id.clone(),
            ));
        }
        if card.declared_status == JurisdictionSupportStatus::Supported {
            match review.disposition {
                CapabilityReviewDisposition::SupportsBoundedSupportClaim => {}
                CapabilityReviewDisposition::ChallengesSupportClaim => {
                    issues.push(CapabilityCardIssue::IndependentReviewerChallengesClaim(
                        review.reviewer_id.clone(),
                    ));
                }
                CapabilityReviewDisposition::Inconclusive => {
                    issues.push(CapabilityCardIssue::IndependentReviewerInconclusive(
                        review.reviewer_id.clone(),
                    ));
                }
            }
        }
    }

    if card.declared_status == JurisdictionSupportStatus::Supported {
        if card.independent_reviews.len() < 2 {
            issues.push(CapabilityCardIssue::InsufficientIndependentReviewers);
        }
        if !card.independent_reviews.iter().any(|review| {
            review.disposition == CapabilityReviewDisposition::SupportsBoundedSupportClaim
        }) {
            issues.push(CapabilityCardIssue::NoIndependentReviewerSupportsClaim);
        }
    }

    let supported_claim_metadata_complete =
        card.declared_status == JurisdictionSupportStatus::Supported && issues.is_empty();
    let status = if !issues.is_empty() {
        CapabilityCardValidationStatus::MetadataIncomplete
    } else {
        match card.declared_status {
            JurisdictionSupportStatus::Draft | JurisdictionSupportStatus::Researching => {
                CapabilityCardValidationStatus::ResearchOnly
            }
            JurisdictionSupportStatus::Pilot => {
                CapabilityCardValidationStatus::PilotMetadataComplete
            }
            JurisdictionSupportStatus::Supported => {
                CapabilityCardValidationStatus::SupportedClaimMetadataComplete
            }
            JurisdictionSupportStatus::Suspended => CapabilityCardValidationStatus::Suspended,
        }
    };

    CapabilityCardReport {
        status,
        issues,
        supported_claim_metadata_complete,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn card(status: JurisdictionSupportStatus) -> JurisdictionCapabilityCard {
        JurisdictionCapabilityCard {
            jurisdiction_id: "ZA/GP/Johannesburg".into(),
            display_name: "Synthetic Johannesburg-area profile".into(),
            declared_status: status,
            card_author_id: "author:pilot".into(),
            subject_sha256: "a".repeat(64),
            source_registry_snapshot_sha256: "b".repeat(64),
            test_corpus_sha256: "c".repeat(64),
            tested_case_count: 120,
            coverage: REQUIRED_REGULATORY_DOMAINS
                .into_iter()
                .map(|domain| DomainCoverageRecord {
                    domain,
                    disposition: DomainCoverageDisposition::Covered {
                        module_ids: vec![format!("synthetic:{}:v1", domain_key(domain))],
                        source_snapshot_sha256: "d".repeat(64),
                    },
                })
                .collect(),
            independent_reviews: vec![
                review(
                    "reviewer-a",
                    CapabilityReviewDisposition::SupportsBoundedSupportClaim,
                ),
                review(
                    "reviewer-b",
                    CapabilityReviewDisposition::SupportsBoundedSupportClaim,
                ),
            ],
            known_scope_gaps: Vec::new(),
            update_policy: "Recheck authority sources and fixture suite on every change notice"
                .into(),
            accountable_maintainer: "maintainer:pilot".into(),
            source_registry_checked_on_day: 120,
            registry_as_of_day: 120,
        }
    }

    fn domain_key(domain: RegulatoryDomain) -> &'static str {
        match domain {
            RegulatoryDomain::ResidentialBuilding => "residential-building",
            RegulatoryDomain::PlanningAndLandUse => "planning-land-use",
            RegulatoryDomain::StructuralSafety => "structural-safety",
            RegulatoryDomain::FireAndLifeSafety => "fire-life-safety",
            RegulatoryDomain::EnergyAndThermal => "energy-thermal",
            RegulatoryDomain::Accessibility => "accessibility",
            RegulatoryDomain::WaterAndSanitation => "water-sanitation",
            RegulatoryDomain::ElectricalInstallations => "electrical-installations",
            RegulatoryDomain::PermitAndInspectionProcess => "permit-inspection",
            RegulatoryDomain::HeritageAndConservation => "heritage-conservation",
            RegulatoryDomain::EnvironmentalAndSiteHazards => "environment-site-hazards",
        }
    }

    fn review(reviewer_id: &str, disposition: CapabilityReviewDisposition) -> CapabilityReview {
        CapabilityReview {
            reviewer_id: reviewer_id.into(),
            review_scope: "Independent source, test, and scope review".into(),
            reviewed_subject_sha256: "a".repeat(64),
            disposition,
            rationale: "Review statement with limitations and supporting references".into(),
            conflict_of_interest_disclosure: "Interests and relationships disclosed".into(),
        }
    }

    #[test]
    fn supported_status_requires_complete_domain_coverage_and_independent_review() {
        let report = validate_capability_card(&card(JurisdictionSupportStatus::Supported), 30);
        assert_eq!(
            report.status,
            CapabilityCardValidationStatus::SupportedClaimMetadataComplete
        );
        assert!(report.supported_claim_metadata_complete);
        assert!(report.issues.is_empty());
    }

    #[test]
    fn a_missing_domain_blocks_supported_claim_but_not_research_card() {
        let mut supported = card(JurisdictionSupportStatus::Supported);
        supported
            .coverage
            .retain(|item| item.domain != RegulatoryDomain::Accessibility);
        let report = validate_capability_card(&supported, 30);
        assert!(!report.supported_claim_metadata_complete);
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::MissingRequiredDomain(
                    RegulatoryDomain::Accessibility
                ))
        );

        supported.declared_status = JurisdictionSupportStatus::Researching;
        let research_report = validate_capability_card(&supported, 30);
        assert!(!research_report.supported_claim_metadata_complete);
        assert!(
            !research_report
                .issues
                .contains(&CapabilityCardIssue::MissingRequiredDomain(
                    RegulatoryDomain::Accessibility
                ))
        );
    }

    #[test]
    fn not_applicable_domain_requires_rationale_and_decision_reference() {
        let mut value = card(JurisdictionSupportStatus::Supported);
        value.coverage[0].disposition = DomainCoverageDisposition::ExplicitlyNotApplicable {
            rationale: " ".into(),
            decision_reference: String::new(),
        };
        let report = validate_capability_card(&value, 30);
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::InvalidNotApplicableRationale {
                    domain: RegulatoryDomain::ResidentialBuilding
                })
        );
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::InvalidNotApplicableDecision {
                    domain: RegulatoryDomain::ResidentialBuilding
                })
        );
    }

    #[test]
    fn stale_registry_snapshot_blocks_supported_claim() {
        let mut value = card(JurisdictionSupportStatus::Supported);
        value.source_registry_checked_on_day = 40;
        let report = validate_capability_card(&value, 30);
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::StaleSourceRegistry)
        );
    }

    #[test]
    fn reviews_must_match_subject_snapshot_and_be_independent() {
        let mut value = card(JurisdictionSupportStatus::Supported);
        value.independent_reviews[0].reviewed_subject_sha256 = "e".repeat(64);
        value.independent_reviews[1].reviewer_id = value.accountable_maintainer.clone();
        let report = validate_capability_card(&value, 30);
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::ReviewDigestMismatch(
                    "reviewer-a".into()
                ))
        );
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::AuthorIncludedAsReviewer(
                    "maintainer:pilot".into()
                ))
        );
    }

    #[test]
    fn a_challenging_review_prevents_a_supported_claim_without_erasing_dissent() {
        let mut value = card(JurisdictionSupportStatus::Supported);
        value.independent_reviews[1].disposition =
            CapabilityReviewDisposition::ChallengesSupportClaim;
        let report = validate_capability_card(&value, 30);
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::IndependentReviewerChallengesClaim(
                    "reviewer-b".into()
                ))
        );
        assert_eq!(
            value.independent_reviews[1].disposition,
            CapabilityReviewDisposition::ChallengesSupportClaim
        );
    }

    #[test]
    fn inconclusive_independent_review_blocks_supported_claim() {
        let mut value = card(JurisdictionSupportStatus::Supported);
        value.independent_reviews[1].disposition = CapabilityReviewDisposition::Inconclusive;
        let report = validate_capability_card(&value, 30);
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::IndependentReviewerInconclusive(
                    "reviewer-b".into()
                ))
        );
        assert!(!report.supported_claim_metadata_complete);
    }

    #[test]
    fn card_author_cannot_count_as_an_independent_reviewer() {
        let mut value = card(JurisdictionSupportStatus::Supported);
        value.independent_reviews[0].reviewer_id = value.card_author_id.clone();
        let report = validate_capability_card(&value, 30);
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::AuthorIncludedAsReviewer(
                    "author:pilot".into()
                ))
        );
        assert!(!report.supported_claim_metadata_complete);
    }

    #[test]
    fn blank_known_scope_gap_is_not_an_auditable_gap_record() {
        let mut value = card(JurisdictionSupportStatus::Researching);
        value.known_scope_gaps.push("  ".into());
        let report = validate_capability_card(&value, 30);
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::KnownGapUndisclosedOrBlank)
        );
    }

    #[test]
    fn duplicate_coverage_and_duplicate_module_ids_are_rejected() {
        let mut value = card(JurisdictionSupportStatus::Supported);
        value.coverage.push(value.coverage[0].clone());
        if let DomainCoverageDisposition::Covered { module_ids, .. } =
            &mut value.coverage[1].disposition
        {
            *module_ids = vec!["duplicate".into(), "duplicate".into()];
        }
        let report = validate_capability_card(&value, 30);
        assert!(
            report
                .issues
                .contains(&CapabilityCardIssue::DuplicateCoverageDomain(
                    RegulatoryDomain::ResidentialBuilding
                ))
        );
    }
}
