//! Semantic support assessment, separate from source provenance.
//!
//! A digest-pinned citation proves source identity, not that the source entails
//! the claim. This crate validates assessment references and requires an
//! independent review before a claim can be classified as supported.
//!
//! It does not parse documents, verify signatures, assess source credibility,
//! or make legal, safety or professional-authority determinations.

#![forbid(unsafe_code)]

use std::collections::{HashMap, HashSet};

/// Current claim-support contract version.
pub const CLAIM_SUPPORT_SCHEMA_VERSION: u16 = 1;

/// Semantic dimensions whose qualifiers may change a claim's meaning.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum SemanticDimensionV1 {
    /// Negation or polarity.
    Negation,
    /// Numeric quantity, threshold or range.
    Quantity,
    /// Measurement unit.
    Unit,
    /// Time, date, freshness or duration.
    TemporalScope,
    /// Population, sample or affected group.
    Population,
    /// Location, system boundary or operational scope.
    Scope,
    /// Legal/technical jurisdiction.
    Jurisdiction,
    /// Confidence, uncertainty or probability.
    Uncertainty,
    /// Causal strength (association, mechanism, causation).
    CausalStrength,
    /// Preconditions, exceptions or operating conditions.
    Conditions,
}

/// Exact claim reference from one proposal revision.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ClaimRefV1 {
    /// Stable claim identifier.
    pub claim_id: String,
    /// Digest of the exact claim text and structured attributes.
    pub claim_digest: [u8; 32],
    /// Digest of the proposal revision containing this claim.
    pub proposal_digest: [u8; 32],
    /// Semantic dimensions that must be checked for this claim.
    pub required_dimensions: Vec<SemanticDimensionV1>,
    /// Qualified independent reviewer roles required for a positive disposition.
    pub required_reviewer_roles: Vec<ReviewerRoleV1>,
}

/// Digest-pinned source excerpt and its locator.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceExcerptRefV1 {
    /// Stable source identifier.
    pub source_id: String,
    /// Digest of the complete source version.
    pub source_digest: [u8; 32],
    /// Digest of the exact passage, observation or measurement selected.
    pub excerpt_digest: [u8; 32],
    /// Digest of the immutable locator (page/span/row/timecode/record key).
    pub locator_digest: [u8; 32],
    /// Revision/version of the source.
    pub source_revision: u64,
}

/// Relationship between a source excerpt and a claim.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportRelationV1 {
    /// Source directly supports the claim as written.
    DirectSupport,
    /// Source supports only part of the claim.
    PartialSupport,
    /// Source contradicts the claim.
    Contradiction,
    /// More context is needed to decide.
    ContextRequired,
    /// The relation cannot be assessed from the available material.
    NotAssessable,
}

/// Result of comparing one semantic qualifier.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QualifierRelationV1 {
    /// Claim and source preserve the same qualifier.
    Preserved,
    /// Claim is narrower than the source and does not exceed its support.
    ClaimNarrowerThanSource,
    /// Claim is stronger than the source.
    ClaimStrongerThanSource,
    /// A source qualifier was omitted from the claim.
    Omitted,
    /// Claim and source qualifiers conflict.
    Contradicted,
    /// The dimension was not assessed.
    NotAssessed,
    /// The dimension genuinely does not apply.
    NotApplicable,
}

/// Comparison of one claim qualifier to the cited excerpt.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct QualifierComparisonV1 {
    /// Semantic dimension checked.
    pub dimension: SemanticDimensionV1,
    /// Digest of the qualifier extracted from the claim.
    pub claim_value_digest: Option<[u8; 32]>,
    /// Digest of the qualifier extracted from the source excerpt.
    pub source_value_digest: Option<[u8; 32]>,
    /// Assessor's comparison.
    pub relation: QualifierRelationV1,
}

/// Origin of a support assessment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssessorKindV1 {
    /// Machine-generated assessment; never self-certifying.
    Machine,
    /// Human domain-expert assessment.
    HumanDomainExpert,
}

/// One semantic support assessment bound to exact claim and source versions.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SupportAssessmentV1 {
    /// Stable assessment identifier.
    pub assessment_id: String,
    /// Exact claim identifier.
    pub claim_id: String,
    /// Exact claim digest assessed.
    pub claim_digest: [u8; 32],
    /// Exact proposal digest assessed.
    pub proposal_digest: [u8; 32],
    /// Exact source identifier.
    pub source_id: String,
    /// Exact source revision assessed.
    pub source_revision: u64,
    /// Exact source digest.
    pub source_digest: [u8; 32],
    /// Exact excerpt digest.
    pub excerpt_digest: [u8; 32],
    /// Exact locator digest.
    pub locator_digest: [u8; 32],
    /// Assessment relation.
    pub relation: SupportRelationV1,
    /// Principal that produced this assessment.
    pub assessor_id: String,
    /// Assessment method identifier.
    pub method_id: String,
    /// Exact method/model version.
    pub method_version: String,
    /// Whether this is a machine or human assessment.
    pub assessor_kind: AssessorKindV1,
    /// Required for machine assessments.
    pub model_digest: Option<[u8; 32]>,
    /// Assessment creation time.
    pub assessed_at: u64,
    /// Optional expiry of the assessment.
    pub valid_until: Option<u64>,
    /// Digest of the complete canonical assessment record.
    pub assessment_digest: [u8; 32],
    /// Qualifier comparisons for the claim.
    pub qualifiers: Vec<QualifierComparisonV1>,
}

/// Local authority reference for an independent review.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LocalAuthorityRefV1 {
    /// Local authority policy identifier.
    pub policy_id: String,
    /// Adopted policy revision.
    pub revision: u64,
    /// Digest of adopted policy.
    pub policy_digest: [u8; 32],
    /// Local decision or delegation record identifier.
    pub decision_id: String,
    /// Digest of local decision or delegation record.
    pub decision_digest: [u8; 32],
}

/// Who is qualified to perform the independent review.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum ReviewerRoleV1 {
    /// Domain expert qualified for the relevant technical domain.
    DomainExpert,
    /// Source owner or custodian authorized to interpret the source.
    SourceAuthority,
    /// Safety authority for safety-relevant claims.
    SafetyAuthority,
}

/// Independent review decision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReviewDecisionV1 {
    /// Confirms direct semantic support.
    ConfirmDirectSupport,
    /// Confirms a material contradiction.
    ConfirmContradiction,
    /// Requires more context or a different source.
    RequestContext,
    /// Rejects this assessment as unreliable or out of scope.
    RejectAssessment,
}

/// Review of one exact assessment digest.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SupportReviewV1 {
    /// Stable review identifier.
    pub review_id: String,
    /// Assessment being reviewed.
    pub assessment_id: String,
    /// Exact assessment digest reviewed.
    pub assessment_digest: [u8; 32],
    /// Independent reviewer principal.
    pub reviewer_id: String,
    /// Reviewer's qualified role.
    pub role: ReviewerRoleV1,
    /// Local authority reference for this review.
    pub authority: LocalAuthorityRefV1,
    /// Review decision.
    pub decision: ReviewDecisionV1,
    /// Review timestamp.
    pub reviewed_at: u64,
    /// Optional expiry.
    pub valid_until: Option<u64>,
    /// Optional effective withdrawal time.
    pub withdrawn_at: Option<u64>,
}

/// A claim-support packet contains all known source assessments for one claim.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ClaimSupportPacketV1 {
    /// Contract schema version.
    pub schema_version: u16,
    /// Exact claim.
    pub claim: ClaimRefV1,
    /// Allowed digest-pinned source excerpts.
    pub sources: Vec<SourceExcerptRefV1>,
    /// Assessments for this claim.
    pub assessments: Vec<SupportAssessmentV1>,
    /// Independent reviews of those assessments.
    pub reviews: Vec<SupportReviewV1>,
}

/// Structural validation errors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClaimSupportErrorV1 {
    /// Unsupported schema version.
    UnsupportedSchema,
    /// Required field is blank or padded.
    EmptyField,
    /// Required digest is missing.
    MissingDigest,
    /// Required semantic dimension is missing.
    MissingQualifierDimension,
    /// Duplicate identifier or semantic dimension.
    DuplicateIdentifier,
    /// Source reference is absent or does not match exact digests.
    SourceReferenceMismatch,
    /// Assessment is not bound to the exact claim/proposal digest.
    ClaimReferenceMismatch,
    /// Assessment qualifier set is malformed.
    InvalidQualifierComparison,
    /// A machine assessment has no pinned model digest.
    MissingModelDigest,
    /// A human assessment must not claim a model digest.
    UnexpectedModelDigest,
    /// Review does not bind to an existing exact assessment.
    ReviewReferenceMismatch,
    /// Reviewer is the same principal as the assessor.
    NonIndependentReview,
    /// Review times are invalid.
    InvalidReviewTime,
    /// Assessment validity interval is invalid.
    InvalidAssessmentTime,
}

/// Result of independent semantic support qualification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClaimSupportDispositionV1 {
    /// All relevant assessments are direct, qualifier-safe and independently confirmed.
    Supported,
    /// A confirmed source/claim contradiction exists.
    BlockedByContradiction,
    /// Evidence is incomplete, ambiguous or needs a new review.
    HumanReviewRequired,
    /// No assessable supporting evidence is available.
    InsufficientEvidence,
}

/// Evaluate a packet only after its references have been structurally validated.
pub fn validate_packet(packet: &ClaimSupportPacketV1) -> Result<(), ClaimSupportErrorV1> {
    if packet.schema_version != CLAIM_SUPPORT_SCHEMA_VERSION {
        return Err(ClaimSupportErrorV1::UnsupportedSchema);
    }
    require_non_empty(&packet.claim.claim_id)?;
    require_digest(&packet.claim.claim_digest)?;
    require_digest(&packet.claim.proposal_digest)?;
    if packet.claim.required_dimensions.is_empty() {
        return Err(ClaimSupportErrorV1::MissingQualifierDimension);
    }
    if packet.claim.required_reviewer_roles.is_empty() {
        return Err(ClaimSupportErrorV1::NonIndependentReview);
    }
    let mut required_roles = HashSet::new();
    for role in &packet.claim.required_reviewer_roles {
        if !required_roles.insert(*role) {
            return Err(ClaimSupportErrorV1::DuplicateIdentifier);
        }
    }

    let mut required = HashSet::new();
    for dimension in &packet.claim.required_dimensions {
        if !required.insert(*dimension) {
            return Err(ClaimSupportErrorV1::DuplicateIdentifier);
        }
    }

    let mut sources = HashMap::new();
    for source in &packet.sources {
        require_non_empty(&source.source_id)?;
        require_digest(&source.source_digest)?;
        require_digest(&source.excerpt_digest)?;
        require_digest(&source.locator_digest)?;
        if source.source_revision == 0 {
            return Err(ClaimSupportErrorV1::SourceReferenceMismatch);
        }
        if sources.insert(source.source_id.as_str(), source).is_some() {
            return Err(ClaimSupportErrorV1::DuplicateIdentifier);
        }
    }

    let mut assessment_ids = HashMap::new();
    for assessment in &packet.assessments {
        require_non_empty(&assessment.assessment_id)?;
        require_non_empty(&assessment.claim_id)?;
        require_non_empty(&assessment.source_id)?;
        require_non_empty(&assessment.assessor_id)?;
        require_non_empty(&assessment.method_id)?;
        require_non_empty(&assessment.method_version)?;
        require_digest(&assessment.claim_digest)?;
        require_digest(&assessment.proposal_digest)?;
        require_digest(&assessment.source_digest)?;
        require_digest(&assessment.excerpt_digest)?;
        require_digest(&assessment.locator_digest)?;
        require_digest(&assessment.assessment_digest)?;

        if assessment.claim_id != packet.claim.claim_id
            || assessment.claim_digest != packet.claim.claim_digest
            || assessment.proposal_digest != packet.claim.proposal_digest
        {
            return Err(ClaimSupportErrorV1::ClaimReferenceMismatch);
        }
        let source = sources
            .get(assessment.source_id.as_str())
            .ok_or(ClaimSupportErrorV1::SourceReferenceMismatch)?;
        if source.source_revision != assessment.source_revision
            || source.source_digest != assessment.source_digest
            || source.excerpt_digest != assessment.excerpt_digest
            || source.locator_digest != assessment.locator_digest
        {
            return Err(ClaimSupportErrorV1::SourceReferenceMismatch);
        }
        match (assessment.assessor_kind, assessment.model_digest) {
            (AssessorKindV1::Machine, Some(digest)) => require_digest(&digest)?,
            (AssessorKindV1::Machine, None) => return Err(ClaimSupportErrorV1::MissingModelDigest),
            (AssessorKindV1::HumanDomainExpert, Some(_)) => {
                return Err(ClaimSupportErrorV1::UnexpectedModelDigest)
            }
            (AssessorKindV1::HumanDomainExpert, None) => {}
        }
        if assessment.valid_until.map(|expiry| expiry < assessment.assessed_at).unwrap_or(false) {
            return Err(ClaimSupportErrorV1::InvalidAssessmentTime);
        }

        let mut compared = HashSet::new();
        for qualifier in &assessment.qualifiers {
            if !required.contains(&qualifier.dimension) || !compared.insert(qualifier.dimension) {
                return Err(ClaimSupportErrorV1::InvalidQualifierComparison);
            }
            match qualifier.relation {
                QualifierRelationV1::NotApplicable => {
                    if qualifier.claim_value_digest.is_some() || qualifier.source_value_digest.is_some() {
                        return Err(ClaimSupportErrorV1::InvalidQualifierComparison);
                    }
                }
                _ => {
                    require_digest(
                        qualifier.claim_value_digest.as_ref().ok_or(
                            ClaimSupportErrorV1::InvalidQualifierComparison,
                        )?,
                    )?;
                    require_digest(
                        qualifier.source_value_digest.as_ref().ok_or(
                            ClaimSupportErrorV1::InvalidQualifierComparison,
                        )?,
                    )?;
                }
            }
        }
        if required.iter().any(|dimension| !compared.contains(dimension)) {
            return Err(ClaimSupportErrorV1::MissingQualifierDimension);
        }
        if assessment_ids.insert(assessment.assessment_id.as_str(), assessment).is_some() {
            return Err(ClaimSupportErrorV1::DuplicateIdentifier);
        }
    }

    let mut review_ids = HashSet::new();
    let mut reviewed_assessments = HashSet::new();
    for review in &packet.reviews {
        require_non_empty(&review.review_id)?;
        require_non_empty(&review.assessment_id)?;
        require_non_empty(&review.reviewer_id)?;
        validate_authority(&review.authority)?;
        require_digest(&review.assessment_digest)?;
        if !review_ids.insert(review.review_id.as_str()) {
            return Err(ClaimSupportErrorV1::DuplicateIdentifier);
        }
        let assessment = assessment_ids
            .get(review.assessment_id.as_str())
            .ok_or(ClaimSupportErrorV1::ReviewReferenceMismatch)?;
        if assessment.assessment_digest != review.assessment_digest {
            return Err(ClaimSupportErrorV1::ReviewReferenceMismatch);
        }
        if assessment.assessor_id == review.reviewer_id {
            return Err(ClaimSupportErrorV1::NonIndependentReview);
        }
        if review.reviewed_at < assessment.assessed_at
            || review.valid_until.map(|expiry| expiry < review.reviewed_at).unwrap_or(false)
            || review.withdrawn_at.map(|time| time < review.reviewed_at).unwrap_or(false)
        {
            return Err(ClaimSupportErrorV1::InvalidReviewTime);
        }
        if !reviewed_assessments.insert((review.assessment_id.as_str(), review.role)) {
            return Err(ClaimSupportErrorV1::DuplicateIdentifier);
        }
    }

    Ok(())
}

fn validate_authority(authority: &LocalAuthorityRefV1) -> Result<(), ClaimSupportErrorV1> {
    require_non_empty(&authority.policy_id)?;
    require_non_empty(&authority.decision_id)?;
    if authority.revision == 0 {
        return Err(ClaimSupportErrorV1::InvalidReviewTime);
    }
    require_digest(&authority.policy_digest)?;
    require_digest(&authority.decision_digest)
}

/// Evaluate a claim's support; a machine assessment alone never returns Supported.
pub fn evaluate_claim_support(
    packet: &ClaimSupportPacketV1,
    now: u64,
) -> Result<ClaimSupportDispositionV1, ClaimSupportErrorV1> {
    validate_packet(packet)?;
    if packet.assessments.is_empty() {
        return Ok(ClaimSupportDispositionV1::InsufficientEvidence);
    }

    let current_reviews = packet
        .reviews
        .iter()
        .filter(|review| {
            review.reviewed_at <= now
                && review.valid_until.map(|expiry| now < expiry).unwrap_or(true)
                && review.withdrawn_at.map(|time| now < time).unwrap_or(true)
        })
        .collect::<Vec<_>>();

    if current_reviews
        .iter()
        .any(|review| review.decision == ReviewDecisionV1::ConfirmContradiction)
    {
        return Ok(ClaimSupportDispositionV1::BlockedByContradiction);
    }

    let mut all_confirmed = true;
    for assessment in &packet.assessments {
        let current = assessment.assessed_at <= now
            && assessment.valid_until.map(|expiry| now < expiry).unwrap_or(true);
        if !current || assessment.relation != SupportRelationV1::DirectSupport {
            all_confirmed = false;
            continue;
        }

        for role in &packet.claim.required_reviewer_roles {
            let review = current_reviews.iter().find(|review| {
                review.assessment_id == assessment.assessment_id && review.role == *role
            });
            let Some(review) = review else {
                all_confirmed = false;
                continue;
            };
            if review.decision != ReviewDecisionV1::ConfirmDirectSupport {
                all_confirmed = false;
            }
        }

        let qualifier_safe = assessment.qualifiers.iter().all(|qualifier| {
            matches!(
                qualifier.relation,
                QualifierRelationV1::Preserved
                    | QualifierRelationV1::ClaimNarrowerThanSource
                    | QualifierRelationV1::NotApplicable
            )
        });
        if !qualifier_safe {
            all_confirmed = false;
        }
    }

    if all_confirmed {
        Ok(ClaimSupportDispositionV1::Supported)
    } else {
        Ok(ClaimSupportDispositionV1::HumanReviewRequired)
    }
}

fn require_non_empty(value: &str) -> Result<(), ClaimSupportErrorV1> {
    if !value.is_empty() && value.trim() == value {
        Ok(())
    } else {
        Err(ClaimSupportErrorV1::EmptyField)
    }
}

fn require_digest(value: &[u8; 32]) -> Result<(), ClaimSupportErrorV1> {
    if value.iter().all(|byte| *byte == 0) {
        Err(ClaimSupportErrorV1::MissingDigest)
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOW: u64 = 100;

    fn authority() -> LocalAuthorityRefV1 {
        LocalAuthorityRefV1 {
            policy_id: "domain-review-policy".to_owned(),
            revision: 1,
            policy_digest: [1; 32],
            decision_id: "review-delegation".to_owned(),
            decision_digest: [2; 32],
        }
    }

    fn packet() -> ClaimSupportPacketV1 {
        let dimensions = vec![
            SemanticDimensionV1::Negation,
            SemanticDimensionV1::Quantity,
            SemanticDimensionV1::Unit,
            SemanticDimensionV1::TemporalScope,
            SemanticDimensionV1::Population,
            SemanticDimensionV1::Scope,
            SemanticDimensionV1::Jurisdiction,
            SemanticDimensionV1::Uncertainty,
            SemanticDimensionV1::CausalStrength,
            SemanticDimensionV1::Conditions,
        ];
        let source = SourceExcerptRefV1 {
            source_id: "pump-service-measurement".to_owned(),
            source_digest: [3; 32],
            excerpt_digest: [4; 32],
            locator_digest: [5; 32],
            source_revision: 1,
        };
        let qualifiers = dimensions.iter().enumerate().map(|(index, dimension)| {
            QualifierComparisonV1 {
                dimension: *dimension,
                claim_value_digest: Some([10 + index as u8; 32]),
                source_value_digest: Some([20 + index as u8; 32]),
                relation: QualifierRelationV1::Preserved,
            }
        }).collect();
        let assessment = SupportAssessmentV1 {
            assessment_id: "assessment-1".to_owned(),
            claim_id: "water-service-hours".to_owned(),
            claim_digest: [6; 32],
            proposal_digest: [7; 32],
            source_id: source.source_id.clone(),
            source_revision: source.source_revision,
            source_digest: source.source_digest,
            excerpt_digest: source.excerpt_digest,
            locator_digest: source.locator_digest,
            relation: SupportRelationV1::DirectSupport,
            assessor_id: "symthaea-assessor".to_owned(),
            method_id: "claim-support-check".to_owned(),
            method_version: "1.0.0".to_owned(),
            assessor_kind: AssessorKindV1::Machine,
            model_digest: Some([8; 32]),
            assessed_at: 10,
            valid_until: Some(200),
            assessment_digest: [9; 32],
            qualifiers,
        };
        ClaimSupportPacketV1 {
            schema_version: CLAIM_SUPPORT_SCHEMA_VERSION,
            claim: ClaimRefV1 {
                claim_id: "water-service-hours".to_owned(),
                claim_digest: [6; 32],
                proposal_digest: [7; 32],
                required_dimensions: dimensions,
                required_reviewer_roles: vec![ReviewerRoleV1::DomainExpert],
            },
            sources: vec![source],
            assessments: vec![assessment],
            reviews: Vec::new(),
        }
    }

    fn review(decision: ReviewDecisionV1) -> SupportReviewV1 {
        SupportReviewV1 {
            review_id: "review-1".to_owned(),
            assessment_id: "assessment-1".to_owned(),
            assessment_digest: [9; 32],
            reviewer_id: "independent-engineer".to_owned(),
            role: ReviewerRoleV1::DomainExpert,
            authority: authority(),
            decision,
            reviewed_at: 20,
            valid_until: Some(180),
            withdrawn_at: None,
        }
    }

    #[test]
    fn direct_support_requires_independent_human_confirmation() {
        let mut packet = packet();
        packet.reviews.push(review(ReviewDecisionV1::ConfirmDirectSupport));
        assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::Supported);
    }

    #[test]
    fn machine_assessment_cannot_self_certify() {
        let packet = packet();
        assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::HumanReviewRequired);
    }

    #[test]
    fn all_required_reviewer_roles_must_confirm_exact_assessment() {
        let mut packet = packet();
        packet.claim.required_reviewer_roles = vec![
            ReviewerRoleV1::DomainExpert,
            ReviewerRoleV1::SafetyAuthority,
        ];
        packet.reviews.push(review(ReviewDecisionV1::ConfirmDirectSupport));
        assert_eq!(
            evaluate_claim_support(&packet, NOW).unwrap(),
            ClaimSupportDispositionV1::HumanReviewRequired
        );
        let mut safety_review = review(ReviewDecisionV1::ConfirmDirectSupport);
        safety_review.review_id = "review-safety".to_owned();
        safety_review.role = ReviewerRoleV1::SafetyAuthority;
        safety_review.reviewer_id = "independent-safety-engineer".to_owned();
        packet.reviews.push(safety_review);
        assert_eq!(
            evaluate_claim_support(&packet, NOW).unwrap(),
            ClaimSupportDispositionV1::Supported
        );
    }

    #[test]
    fn missing_reviewer_role_requirement_is_rejected() {
        let mut packet = packet();
        packet.claim.required_reviewer_roles.clear();
        assert_eq!(
            validate_packet(&packet),
            Err(ClaimSupportErrorV1::NonIndependentReview)
        );
    }

    #[test]
    fn duplicate_reviewer_role_decisions_for_one_assessment_are_rejected() {
        let mut packet = packet();
        packet.reviews.push(review(ReviewDecisionV1::ConfirmDirectSupport));
        let mut duplicate = review(ReviewDecisionV1::ConfirmDirectSupport);
        duplicate.review_id = "review-duplicate".to_owned();
        packet.reviews.push(duplicate);
        assert_eq!(
            validate_packet(&packet),
            Err(ClaimSupportErrorV1::DuplicateIdentifier)
        );
    }

    #[test]
    fn source_digest_mismatch_is_rejected() {
        let mut packet = packet();
        packet.assessments[0].source_digest = [11; 32];
        assert_eq!(validate_packet(&packet), Err(ClaimSupportErrorV1::SourceReferenceMismatch));
    }

    #[test]
    fn changed_source_revision_invalidates_semantic_assessment() {
        let mut packet = packet();
        packet.assessments[0].source_revision = 2;
        assert_eq!(
            validate_packet(&packet),
            Err(ClaimSupportErrorV1::SourceReferenceMismatch)
        );
    }

    #[test]
    fn future_dated_assessment_cannot_support_claim() {
        let mut packet = packet();
        packet.assessments[0].assessed_at = NOW + 1;
        packet.reviews.clear();
        assert_eq!(
            evaluate_claim_support(&packet, NOW).unwrap(),
            ClaimSupportDispositionV1::HumanReviewRequired
        );
    }

    #[test]
    fn request_context_never_counts_as_positive_confirmation() {
        let mut packet = packet();
        packet.reviews.push(review(ReviewDecisionV1::RequestContext));
        assert_eq!(
            evaluate_claim_support(&packet, NOW).unwrap(),
            ClaimSupportDispositionV1::HumanReviewRequired
        );
    }

    #[test]
    fn one_confirmed_contradiction_blocks_other_positive_sources() {
        let mut packet = packet();
        let mut second_source = packet.sources[0].clone();
        second_source.source_id = "contradictory-measurement".to_owned();
        second_source.source_digest = [31; 32];
        second_source.excerpt_digest = [32; 32];
        second_source.locator_digest = [33; 32];
        second_source.source_revision = 1;
        packet.sources.push(second_source.clone());
        let mut contradiction = packet.assessments[0].clone();
        contradiction.assessment_id = "assessment-contradiction".to_owned();
        contradiction.source_id = second_source.source_id;
        contradiction.source_revision = second_source.source_revision;
        contradiction.source_digest = second_source.source_digest;
        contradiction.excerpt_digest = second_source.excerpt_digest;
        contradiction.locator_digest = second_source.locator_digest;
        contradiction.relation = SupportRelationV1::Contradiction;
        contradiction.assessment_digest = [34; 32];
        packet.assessments.push(contradiction);
        let mut contradiction_review = review(ReviewDecisionV1::ConfirmContradiction);
        contradiction_review.review_id = "review-contradiction".to_owned();
        contradiction_review.assessment_id = "assessment-contradiction".to_owned();
        contradiction_review.assessment_digest = [34; 32];
        contradiction_review.reviewer_id = "independent-source-owner".to_owned();
        contradiction_review.role = ReviewerRoleV1::SourceAuthority;
        packet.claim.required_reviewer_roles.push(ReviewerRoleV1::SourceAuthority);
        packet.reviews.push(contradiction_review);
        packet.reviews.push(review(ReviewDecisionV1::ConfirmDirectSupport));
        assert_eq!(
            evaluate_claim_support(&packet, NOW).unwrap(),
            ClaimSupportDispositionV1::BlockedByContradiction
        );
    }

    #[test]
    fn changed_claim_digest_invalidates_assessment() {
        let mut packet = packet();
        packet.claim.claim_digest = [12; 32];
        assert_eq!(validate_packet(&packet), Err(ClaimSupportErrorV1::ClaimReferenceMismatch));
    }

    #[test]
    fn all_required_qualifier_dimensions_must_be_checked() {
        let mut packet = packet();
        packet.assessments[0].qualifiers.pop();
        assert_eq!(validate_packet(&packet), Err(ClaimSupportErrorV1::MissingQualifierDimension));
    }

    #[test]
    fn strengthened_claim_qualifier_never_counts_as_direct_support() {
        let mut packet = packet();
        packet.assessments[0].qualifiers[1].relation = QualifierRelationV1::ClaimStrongerThanSource;
        packet.reviews.push(review(ReviewDecisionV1::ConfirmDirectSupport));
        assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::HumanReviewRequired);
    }

    #[test]
    fn omitted_or_contradictory_qualifier_requires_review() {
        for relation in [QualifierRelationV1::Omitted, QualifierRelationV1::Contradicted] {
            let mut packet = packet();
            packet.assessments[0].qualifiers[0].relation = relation;
            packet.reviews.push(review(ReviewDecisionV1::ConfirmDirectSupport));
            assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::HumanReviewRequired);
        }
    }

    #[test]
    fn confirmed_contradiction_blocks_support() {
        let mut packet = packet();
        packet.assessments[0].relation = SupportRelationV1::Contradiction;
        packet.reviews.push(review(ReviewDecisionV1::ConfirmContradiction));
        assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::BlockedByContradiction);
    }

    #[test]
    fn unreviewed_contradiction_requires_human_review() {
        let mut packet = packet();
        packet.assessments[0].relation = SupportRelationV1::Contradiction;
        assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::HumanReviewRequired);
    }

    #[test]
    fn partial_support_cannot_be_promoted_to_direct_support() {
        let mut packet = packet();
        packet.assessments[0].relation = SupportRelationV1::PartialSupport;
        packet.reviews.push(review(ReviewDecisionV1::ConfirmDirectSupport));
        assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::HumanReviewRequired);
    }

    #[test]
    fn context_required_and_not_assessable_abstain() {
        for relation in [SupportRelationV1::ContextRequired, SupportRelationV1::NotAssessable] {
            let mut packet = packet();
            packet.assessments[0].relation = relation;
            packet.reviews.push(review(ReviewDecisionV1::ConfirmDirectSupport));
            assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::HumanReviewRequired);
        }
    }

    #[test]
    fn model_assessment_requires_a_pinned_model_digest() {
        let mut packet = packet();
        packet.assessments[0].model_digest = None;
        assert_eq!(validate_packet(&packet), Err(ClaimSupportErrorV1::MissingModelDigest));
    }

    #[test]
    fn human_assessment_cannot_claim_a_model_digest() {
        let mut packet = packet();
        packet.assessments[0].assessor_kind = AssessorKindV1::HumanDomainExpert;
        assert_eq!(validate_packet(&packet), Err(ClaimSupportErrorV1::UnexpectedModelDigest));
    }

    #[test]
    fn assessor_cannot_review_their_own_assessment() {
        let mut packet = packet();
        let mut review = review(ReviewDecisionV1::ConfirmDirectSupport);
        review.reviewer_id = "symthaea-assessor".to_owned();
        packet.reviews.push(review);
        assert_eq!(validate_packet(&packet), Err(ClaimSupportErrorV1::NonIndependentReview));
    }

    #[test]
    fn stale_review_does_not_confirm_support() {
        let mut packet = packet();
        let mut review = review(ReviewDecisionV1::ConfirmDirectSupport);
        review.valid_until = Some(50);
        packet.reviews.push(review);
        assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::HumanReviewRequired);
    }

    #[test]
    fn wrong_assessment_digest_does_not_confirm_support() {
        let mut packet = packet();
        let mut review = review(ReviewDecisionV1::ConfirmDirectSupport);
        review.assessment_digest = [13; 32];
        packet.reviews.push(review);
        assert_eq!(validate_packet(&packet), Err(ClaimSupportErrorV1::ReviewReferenceMismatch));
    }

    #[test]
    fn empty_packet_reports_insufficient_evidence() {
        let mut packet = packet();
        packet.assessments.clear();
        packet.reviews.clear();
        assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::InsufficientEvidence);
    }

    #[test]
    fn not_applicable_qualifier_has_no_value_digests() {
        let mut packet = packet();
        packet.assessments[0].qualifiers[9] = QualifierComparisonV1 {
            dimension: SemanticDimensionV1::Conditions,
            claim_value_digest: None,
            source_value_digest: None,
            relation: QualifierRelationV1::NotApplicable,
        };
        packet.reviews.push(review(ReviewDecisionV1::ConfirmDirectSupport));
        assert_eq!(evaluate_claim_support(&packet, NOW).unwrap(), ClaimSupportDispositionV1::Supported);
    }

    #[test]
    fn duplicate_qualifier_dimension_is_rejected() {
        let mut packet = packet();
        packet.assessments[0].qualifiers[1].dimension = SemanticDimensionV1::Negation;
        assert_eq!(validate_packet(&packet), Err(ClaimSupportErrorV1::InvalidQualifierComparison));
    }
}
