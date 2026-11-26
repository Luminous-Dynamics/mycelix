#![forbid(unsafe_code)]

//! Context-bound impact determinations and declared evidence sufficiency.
//!
//! This profile consumes owner-qualified event, lease, provenance and review
//! references. It does not create event storage, temporal authority, leases,
//! source truth, semantic claim support or decision authority.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub type DigestV1 = [u8; 32];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ArtifactVersionRefV1 {
    pub artifact_id: String,
    pub revision: u64,
    pub digest: DigestV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChangeEventRefV1 {
    pub event_id: String,
    pub digest: DigestV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DependencyEdgeRefV1 {
    pub edge_id: String,
    pub digest: DigestV1,
    pub upstream: ArtifactVersionRefV1,
    pub downstream: ArtifactVersionRefV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UseContextRefV1 {
    pub context_id: String,
    pub digest: DigestV1,
    pub jurisdiction: String,
    pub population_scope_digest: DigestV1,
    pub use_scope_digest: DigestV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OwnerQualifiedLeaseRefV1 {
    pub lease_id: String,
    pub digest: DigestV1,
    pub verified_at: u64,
    pub valid_until: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImpactStatusV1 {
    Affected,
    NotAffected,
    UnderInvestigation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImpactJustificationV1 {
    ComponentNotPresent,
    NotInExecutionPath,
    ScopeExcludesChangedArea,
    BoundedMitigation,
    DirectlyAffected,
    InsufficientInformation,
    ConflictingEvidence,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImpactReviewerRoleV1 {
    DomainAuthority,
    SafetyAuthority,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ImpactReviewV1 {
    pub reviewer_id: String,
    pub role: ImpactReviewerRoleV1,
    pub approved_determination_digest: DigestV1,
    pub authority_ref: ArtifactVersionRefV1,
    pub reviewed_at: u64,
    pub lease: OwnerQualifiedLeaseRefV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ImpactEvidenceRefV1 {
    pub evidence_id: String,
    pub digest: DigestV1,
    pub context_digest: DigestV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ImpactDeterminationV1 {
    pub determination_id: String,
    pub digest: DigestV1,
    pub change_event: ChangeEventRefV1,
    pub changed_artifact: ArtifactVersionRefV1,
    pub dependent_artifact: ArtifactVersionRefV1,
    pub dependency_edge: DependencyEdgeRefV1,
    pub context: UseContextRefV1,
    pub status: ImpactStatusV1,
    pub justification: ImpactJustificationV1,
    pub evidence: Vec<ImpactEvidenceRefV1>,
    pub assessor_id: String,
    pub issued_at: u64,
    /// Opaque owner-qualified lease. This crate checks but never composes it.
    pub lease: OwnerQualifiedLeaseRefV1,
    pub reviews: Vec<ImpactReviewV1>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExpectedImpactScopeV1 {
    pub change_event: ChangeEventRefV1,
    pub changed_artifact: ArtifactVersionRefV1,
    pub dependent_artifact: ArtifactVersionRefV1,
    pub dependency_edge: DependencyEdgeRefV1,
    pub context: UseContextRefV1,
    pub domain_authority: ArtifactVersionRefV1,
    pub safety_authority: Option<ArtifactVersionRefV1>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ImpactDecisionV1 {
    Affected,
    NotAffected { justification: ImpactJustificationV1 },
    UnderInvestigation { high_consequence_block: bool },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImpactQualificationErrorV1 {
    InvalidIdentifier,
    InvalidDigest,
    InvalidArtifactReference,
    InvalidContext,
    InvalidLease,
    LeaseNotCurrent,
    EventScopeMismatch,
    ArtifactScopeMismatch,
    DependencyEdgeMismatch,
    ContextScopeMismatch,
    InvalidStatusJustification,
    MissingImpactEvidence,
    InvalidReview,
    DuplicateReviewer,
    MissingDomainReview,
    MissingSafetyReview,
    DuplicateEvidence,
}

impl fmt::Display for ImpactQualificationErrorV1 {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for ImpactQualificationErrorV1 {}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty() && value.trim() == value
}

fn valid_digest(digest: &DigestV1) -> bool {
    *digest != [0; 32]
}

fn valid_artifact(reference: &ArtifactVersionRefV1) -> bool {
    non_empty(&reference.artifact_id)
        && reference.revision > 0
        && valid_digest(&reference.digest)
}

fn valid_lease(lease: &OwnerQualifiedLeaseRefV1) -> bool {
    non_empty(&lease.lease_id)
        && valid_digest(&lease.digest)
        && lease.verified_at > 0
        && lease.valid_until > lease.verified_at
}

fn lease_is_current(lease: &OwnerQualifiedLeaseRefV1, now: u64) -> bool {
    valid_lease(lease) && lease.verified_at <= now && now < lease.valid_until
}

fn valid_context(context: &UseContextRefV1) -> bool {
    non_empty(&context.context_id)
        && valid_digest(&context.digest)
        && non_empty(&context.jurisdiction)
        && valid_digest(&context.population_scope_digest)
        && valid_digest(&context.use_scope_digest)
}

fn valid_event(event: &ChangeEventRefV1) -> bool {
    non_empty(&event.event_id) && valid_digest(&event.digest)
}

fn valid_edge(edge: &DependencyEdgeRefV1) -> bool {
    non_empty(&edge.edge_id)
        && valid_digest(&edge.digest)
        && valid_artifact(&edge.upstream)
        && valid_artifact(&edge.downstream)
}

fn validate_scope(
    determination: &ImpactDeterminationV1,
    expected: &ExpectedImpactScopeV1,
) -> Result<(), ImpactQualificationErrorV1> {
    if !valid_event(&determination.change_event)
        || !valid_event(&expected.change_event)
        || !valid_digest(&determination.digest)
        || !non_empty(&determination.determination_id)
        || !non_empty(&determination.assessor_id)
    {
        return Err(ImpactQualificationErrorV1::InvalidIdentifier);
    }
    if !valid_artifact(&determination.changed_artifact)
        || !valid_artifact(&determination.dependent_artifact)
        || !valid_artifact(&expected.changed_artifact)
        || !valid_artifact(&expected.dependent_artifact)
    {
        return Err(ImpactQualificationErrorV1::InvalidArtifactReference);
    }
    if !valid_edge(&determination.dependency_edge)
        || !valid_edge(&expected.dependency_edge)
    {
        return Err(ImpactQualificationErrorV1::DependencyEdgeMismatch);
    }
    if !valid_context(&determination.context) || !valid_context(&expected.context) {
        return Err(ImpactQualificationErrorV1::InvalidContext);
    }
    if !valid_artifact(&expected.domain_authority)
        || expected
            .safety_authority
            .as_ref()
            .map_or(false, |authority| !valid_artifact(authority))
    {
        return Err(ImpactQualificationErrorV1::InvalidArtifactReference);
    }
    if determination.change_event != expected.change_event {
        return Err(ImpactQualificationErrorV1::EventScopeMismatch);
    }
    if determination.changed_artifact != expected.changed_artifact
        || determination.dependent_artifact != expected.dependent_artifact
    {
        return Err(ImpactQualificationErrorV1::ArtifactScopeMismatch);
    }
    if determination.dependency_edge != expected.dependency_edge
        || determination.dependency_edge.upstream != determination.changed_artifact
        || determination.dependency_edge.downstream != determination.dependent_artifact
    {
        return Err(ImpactQualificationErrorV1::DependencyEdgeMismatch);
    }
    if determination.context != expected.context {
        return Err(ImpactQualificationErrorV1::ContextScopeMismatch);
    }
    Ok(())
}

fn validate_status_justification(
    status: ImpactStatusV1,
    justification: ImpactJustificationV1,
) -> Result<(), ImpactQualificationErrorV1> {
    let allowed = match status {
        ImpactStatusV1::Affected => justification == ImpactJustificationV1::DirectlyAffected,
        ImpactStatusV1::NotAffected => matches!(
            justification,
            ImpactJustificationV1::ComponentNotPresent
                | ImpactJustificationV1::NotInExecutionPath
                | ImpactJustificationV1::ScopeExcludesChangedArea
                | ImpactJustificationV1::BoundedMitigation
        ),
        ImpactStatusV1::UnderInvestigation => matches!(
            justification,
            ImpactJustificationV1::InsufficientInformation
                | ImpactJustificationV1::ConflictingEvidence
        ),
    };
    if allowed {
        Ok(())
    } else {
        Err(ImpactQualificationErrorV1::InvalidStatusJustification)
    }
}

/// Validate a scoped impact determination against one exact downstream use.
///
/// A valid NotAffected result exempts only the named edge for the exact event
/// and context. High-consequence uses also require a distinct SafetyAuthority.
pub fn evaluate_impact_determination(
    determination: &ImpactDeterminationV1,
    expected: &ExpectedImpactScopeV1,
    now: u64,
    high_consequence: bool,
) -> Result<ImpactDecisionV1, ImpactQualificationErrorV1> {
    validate_scope(determination, expected)?;
    validate_status_justification(determination.status, determination.justification)?;

    if determination.issued_at == 0 || determination.issued_at > now {
        return Err(ImpactQualificationErrorV1::InvalidLease);
    }
    if !lease_is_current(&determination.lease, now) {
        return Err(ImpactQualificationErrorV1::LeaseNotCurrent);
    }

    let mut reviewers = BTreeSet::new();
    let mut has_domain_review = false;
    let mut has_safety_review = false;
    for review in &determination.reviews {
        if !non_empty(&review.reviewer_id)
            || review.reviewer_id == determination.assessor_id
            || !valid_artifact(&review.authority_ref)
            || !valid_digest(&review.approved_determination_digest)
            || review.approved_determination_digest != determination.digest
            || review.reviewed_at < determination.issued_at
            || review.reviewed_at > now
            || !lease_is_current(&review.lease, now)
        {
            return Err(ImpactQualificationErrorV1::InvalidReview);
        }
        if !reviewers.insert(review.reviewer_id.clone()) {
            return Err(ImpactQualificationErrorV1::DuplicateReviewer);
        }
        match review.role {
            ImpactReviewerRoleV1::DomainAuthority => {
                if review.authority_ref != expected.domain_authority {
                    return Err(ImpactQualificationErrorV1::InvalidReview);
                }
                has_domain_review = true;
            }
            ImpactReviewerRoleV1::SafetyAuthority => {
                if expected.safety_authority.as_ref() != Some(&review.authority_ref) {
                    return Err(ImpactQualificationErrorV1::InvalidReview);
                }
                has_safety_review = true;
            }
        }
    }

    if !has_domain_review {
        return Err(ImpactQualificationErrorV1::MissingDomainReview);
    }
    if high_consequence && !has_safety_review {
        return Err(ImpactQualificationErrorV1::MissingSafetyReview);
    }

    let mut evidence_ids = BTreeSet::new();
    for evidence in &determination.evidence {
        if !non_empty(&evidence.evidence_id) || !valid_digest(&evidence.digest) {
            return Err(ImpactQualificationErrorV1::InvalidIdentifier);
        }
        if evidence.context_digest != determination.context.digest {
            return Err(ImpactQualificationErrorV1::ContextScopeMismatch);
        }
        if !evidence_ids.insert(evidence.evidence_id.clone()) {
            return Err(ImpactQualificationErrorV1::DuplicateEvidence);
        }
    }

    match determination.status {
        ImpactStatusV1::Affected => {
            if determination.evidence.is_empty() {
                return Err(ImpactQualificationErrorV1::MissingImpactEvidence);
            }
            Ok(ImpactDecisionV1::Affected)
        }
        ImpactStatusV1::NotAffected => {
            if determination.evidence.is_empty() {
                return Err(ImpactQualificationErrorV1::MissingImpactEvidence);
            }
            Ok(ImpactDecisionV1::NotAffected {
                justification: determination.justification,
            })
        }
        ImpactStatusV1::UnderInvestigation => Ok(ImpactDecisionV1::UnderInvestigation {
            high_consequence_block: high_consequence,
        }),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SupportEvidenceStateV1 {
    DirectSupport,
    PartialSupport,
    Contradicted,
    Stale,
    NotAssessable,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupportEvidenceV1 {
    pub evidence_id: String,
    pub digest: DigestV1,
    /// Assigned by a qualified source/provenance assessor, not inferred from IDs.
    pub independence_group_id: Option<String>,
    /// Normally the qualified disposition from Claim Support.
    pub state: SupportEvidenceStateV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SupportRuleV1 {
    AllOf { required_evidence_ids: Vec<String> },
    AnyOf { alternative_support_sets: Vec<Vec<String>> },
    AtLeastIndependent {
        candidate_evidence_ids: Vec<String>,
        minimum: usize,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SupportSufficiencyV1 {
    Supported,
    InsufficientEvidence,
    ConflictReviewRequired,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SupportSufficiencyErrorV1 {
    EmptyEvidenceSet,
    InvalidEvidenceReference,
    DuplicateEvidence,
    InvalidRule,
    UnknownEvidence,
    DuplicateRuleMember,
    MissingIndependenceGroup,
}

impl fmt::Display for SupportSufficiencyErrorV1 {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for SupportSufficiencyErrorV1 {}

fn validate_members(
    members: &[String],
    evidence: &BTreeMap<String, &SupportEvidenceV1>,
) -> Result<(), SupportSufficiencyErrorV1> {
    if members.is_empty() {
        return Err(SupportSufficiencyErrorV1::InvalidRule);
    }
    let mut seen = BTreeSet::new();
    for member in members {
        if !non_empty(member) {
            return Err(SupportSufficiencyErrorV1::InvalidRule);
        }
        if !seen.insert(member) {
            return Err(SupportSufficiencyErrorV1::DuplicateRuleMember);
        }
        if !evidence.contains_key(member) {
            return Err(SupportSufficiencyErrorV1::UnknownEvidence);
        }
    }
    Ok(())
}

/// Evaluate a declared sufficiency rule over owner-qualified claim-support results.
///
/// Contradictory candidate evidence forces review instead of being averaged away.
/// K-of-n counts distinct common-cause groups, never source/document/model IDs.
pub fn evaluate_support_sufficiency(
    rule: &SupportRuleV1,
    items: &[SupportEvidenceV1],
) -> Result<SupportSufficiencyV1, SupportSufficiencyErrorV1> {
    if items.is_empty() {
        return Err(SupportSufficiencyErrorV1::EmptyEvidenceSet);
    }

    let mut evidence = BTreeMap::new();
    for item in items {
        if !non_empty(&item.evidence_id) || !valid_digest(&item.digest) {
            return Err(SupportSufficiencyErrorV1::InvalidEvidenceReference);
        }
        if item.independence_group_id.as_ref().map_or(false, |group| !non_empty(group)) {
            return Err(SupportSufficiencyErrorV1::InvalidEvidenceReference);
        }
        if evidence.insert(item.evidence_id.clone(), item).is_some() {
            return Err(SupportSufficiencyErrorV1::DuplicateEvidence);
        }
    }

    let candidate_ids = match rule {
        SupportRuleV1::AllOf { required_evidence_ids } => required_evidence_ids,
        SupportRuleV1::AnyOf { alternative_support_sets } => {
            if alternative_support_sets.is_empty() {
                return Err(SupportSufficiencyErrorV1::InvalidRule);
            }
            let mut union = BTreeSet::new();
            for alternative in alternative_support_sets {
                validate_members(alternative, &evidence)?;
                union.extend(alternative.iter().cloned());
            }
            return evaluate_candidates(
                &evidence,
                &union.into_iter().collect::<Vec<_>>(),
                rule,
            );
        }
        SupportRuleV1::AtLeastIndependent {
            candidate_evidence_ids,
            minimum,
        } => {
            if *minimum == 0 || *minimum > candidate_evidence_ids.len() {
                return Err(SupportSufficiencyErrorV1::InvalidRule);
            }
            candidate_evidence_ids
        }
    };

    validate_members(candidate_ids, &evidence)?;
    evaluate_candidates(&evidence, candidate_ids, rule)
}

fn evaluate_candidates(
    evidence: &BTreeMap<String, &SupportEvidenceV1>,
    candidates: &[String],
    rule: &SupportRuleV1,
) -> Result<SupportSufficiencyV1, SupportSufficiencyErrorV1> {
    if candidates.iter().any(|id| {
        evidence
            .get(id)
            .map_or(false, |item| item.state == SupportEvidenceStateV1::Contradicted)
    }) {
        return Ok(SupportSufficiencyV1::ConflictReviewRequired);
    }

    match rule {
        SupportRuleV1::AllOf { required_evidence_ids } => {
            let all_supported = required_evidence_ids.iter().all(|id| {
                evidence
                    .get(id)
                    .map_or(false, |item| item.state == SupportEvidenceStateV1::DirectSupport)
            });
            Ok(if all_supported {
                SupportSufficiencyV1::Supported
            } else {
                SupportSufficiencyV1::InsufficientEvidence
            })
        }
        SupportRuleV1::AnyOf { alternative_support_sets } => {
            let supported_alternative = alternative_support_sets.iter().any(|set| {
                set.iter().all(|id| {
                    evidence
                        .get(id)
                        .map_or(false, |item| item.state == SupportEvidenceStateV1::DirectSupport)
                })
            });
            Ok(if supported_alternative {
                SupportSufficiencyV1::Supported
            } else {
                SupportSufficiencyV1::InsufficientEvidence
            })
        }
        SupportRuleV1::AtLeastIndependent {
            candidate_evidence_ids,
            minimum,
        } => {
            let mut groups = BTreeSet::new();
            for id in candidate_evidence_ids {
                let Some(item) = evidence.get(id) else {
                    return Err(SupportSufficiencyErrorV1::UnknownEvidence);
                };
                if item.state == SupportEvidenceStateV1::DirectSupport {
                    let Some(group) = &item.independence_group_id else {
                        return Err(SupportSufficiencyErrorV1::MissingIndependenceGroup);
                    };
                    groups.insert(group.clone());
                }
            }
            Ok(if groups.len() >= *minimum {
                SupportSufficiencyV1::Supported
            } else {
                SupportSufficiencyV1::InsufficientEvidence
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOW: u64 = 1_800_000_100;

    fn digest(byte: u8) -> DigestV1 {
        [byte; 32]
    }

    fn artifact(id: &str, revision: u64, byte: u8) -> ArtifactVersionRefV1 {
        ArtifactVersionRefV1 {
            artifact_id: id.to_owned(),
            revision,
            digest: digest(byte),
        }
    }

    fn lease(id: &str) -> OwnerQualifiedLeaseRefV1 {
        OwnerQualifiedLeaseRefV1 {
            lease_id: id.to_owned(),
            digest: digest(90),
            verified_at: NOW - 50,
            valid_until: NOW + 50,
        }
    }

    fn expected_scope() -> ExpectedImpactScopeV1 {
        let source = artifact("source", 3, 3);
        let downstream = artifact("decision", 4, 4);
        ExpectedImpactScopeV1 {
            change_event: ChangeEventRefV1 {
                event_id: "event-7".to_owned(),
                digest: digest(6),
            },
            changed_artifact: source.clone(),
            dependent_artifact: downstream.clone(),
            dependency_edge: DependencyEdgeRefV1 {
                edge_id: "decision-source".to_owned(),
                digest: digest(5),
                upstream: source,
                downstream,
            },
            context: UseContextRefV1 {
                context_id: "project-scope".to_owned(),
                digest: digest(7),
                jurisdiction: "ZA".to_owned(),
                population_scope_digest: digest(8),
                use_scope_digest: digest(9),
            },
            domain_authority: artifact("domain-charter", 2, 12),
            safety_authority: Some(artifact("safety-charter", 1, 13)),
        }
    }

    fn determination(status: ImpactStatusV1) -> ImpactDeterminationV1 {
        let expected = expected_scope();
        let justification = match status {
            ImpactStatusV1::Affected => ImpactJustificationV1::DirectlyAffected,
            ImpactStatusV1::NotAffected => ImpactJustificationV1::NotInExecutionPath,
            ImpactStatusV1::UnderInvestigation => {
                ImpactJustificationV1::InsufficientInformation
            }
        };
        ImpactDeterminationV1 {
            determination_id: "impact-1".to_owned(),
            digest: digest(10),
            change_event: expected.change_event.clone(),
            changed_artifact: expected.changed_artifact.clone(),
            dependent_artifact: expected.dependent_artifact.clone(),
            dependency_edge: expected.dependency_edge.clone(),
            context: expected.context.clone(),
            status,
            justification,
            evidence: vec![ImpactEvidenceRefV1 {
                evidence_id: "execution-map".to_owned(),
                digest: digest(11),
                context_digest: expected.context.digest,
            }],
            assessor_id: "assessor".to_owned(),
            issued_at: NOW - 20,
            lease: lease("impact-lease"),
            reviews: vec![
                ImpactReviewV1 {
                    reviewer_id: "domain-reviewer".to_owned(),
                    role: ImpactReviewerRoleV1::DomainAuthority,
                    approved_determination_digest: digest(10),
                    authority_ref: expected.domain_authority.clone(),
                    reviewed_at: NOW - 10,
                    lease: lease("domain-review-lease"),
                },
                ImpactReviewV1 {
                    reviewer_id: "safety-reviewer".to_owned(),
                    role: ImpactReviewerRoleV1::SafetyAuthority,
                    approved_determination_digest: digest(10),
                    authority_ref: expected.safety_authority.clone().expect("safety authority ref"),
                    reviewed_at: NOW - 9,
                    lease: lease("safety-review-lease"),
                },
            ],
        }
    }

    fn item(
        id: &str,
        state: SupportEvidenceStateV1,
        group: Option<&str>,
        byte: u8,
    ) -> SupportEvidenceV1 {
        SupportEvidenceV1 {
            evidence_id: id.to_owned(),
            digest: digest(byte),
            independence_group_id: group.map(str::to_owned),
            state,
        }
    }

    #[test]
    fn reviewed_not_affected_packet_is_bound_to_exact_edge_and_context() {
        let expected = expected_scope();
        let packet = determination(ImpactStatusV1::NotAffected);
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, true),
            Ok(ImpactDecisionV1::NotAffected {
                justification: ImpactJustificationV1::NotInExecutionPath
            })
        );
    }

    #[test]
    fn changed_event_or_context_cannot_reuse_not_affected_packet() {
        let mut expected = expected_scope();
        expected.change_event.digest = digest(44);
        let packet = determination(ImpactStatusV1::NotAffected);
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, false),
            Err(ImpactQualificationErrorV1::EventScopeMismatch)
        );

        let mut expected = expected_scope();
        expected.context.population_scope_digest = digest(45);
        let packet = determination(ImpactStatusV1::NotAffected);
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, false),
            Err(ImpactQualificationErrorV1::ContextScopeMismatch)
        );
    }

    #[test]
    fn high_consequence_not_affected_requires_safety_review() {
        let expected = expected_scope();
        let mut packet = determination(ImpactStatusV1::NotAffected);
        packet.reviews.pop();
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, true),
            Err(ImpactQualificationErrorV1::MissingSafetyReview)
        );
    }

    #[test]
    fn reviewer_must_be_bound_to_exact_authority_revision() {
        let expected = expected_scope();
        let mut packet = determination(ImpactStatusV1::NotAffected);
        packet.reviews[0].authority_ref = artifact("domain-charter", 2, 99);
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, false),
            Err(ImpactQualificationErrorV1::InvalidReview)
        );
    }

    #[test]
    fn impact_evidence_must_be_bound_to_exact_use_context() {
        let expected = expected_scope();
        let mut packet = determination(ImpactStatusV1::NotAffected);
        packet.evidence[0].context_digest = digest(99);
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, false),
            Err(ImpactQualificationErrorV1::ContextScopeMismatch)
        );
    }

    #[test]
    fn assessor_cannot_approve_own_not_affected_packet() {
        let expected = expected_scope();
        let mut packet = determination(ImpactStatusV1::NotAffected);
        packet.reviews[0].reviewer_id = packet.assessor_id.clone();
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, false),
            Err(ImpactQualificationErrorV1::InvalidReview)
        );
    }

    #[test]
    fn expired_or_future_lease_cannot_support_not_affected() {
        let expected = expected_scope();
        let mut packet = determination(ImpactStatusV1::NotAffected);
        packet.lease.valid_until = NOW;
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, false),
            Err(ImpactQualificationErrorV1::LeaseNotCurrent)
        );

        let mut packet = determination(ImpactStatusV1::NotAffected);
        packet.reviews[0].lease.verified_at = NOW + 1;
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, false),
            Err(ImpactQualificationErrorV1::InvalidReview)
        );
    }

    #[test]
    fn under_investigation_never_suppresses_high_consequence_edge() {
        let expected = expected_scope();
        let packet = determination(ImpactStatusV1::UnderInvestigation);
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, true),
            Ok(ImpactDecisionV1::UnderInvestigation {
                high_consequence_block: true
            })
        );
    }

    #[test]
    fn affected_packet_is_never_an_exemption() {
        let expected = expected_scope();
        let packet = determination(ImpactStatusV1::Affected);
        assert_eq!(
            evaluate_impact_determination(&packet, &expected, NOW, true),
            Ok(ImpactDecisionV1::Affected)
        );
    }

    #[test]
    fn any_of_accepts_a_surviving_complete_alternative_set() {
        let evidence = vec![
            item("a", SupportEvidenceStateV1::Stale, Some("origin-a"), 1),
            item("b", SupportEvidenceStateV1::DirectSupport, Some("origin-b"), 2),
            item("c", SupportEvidenceStateV1::PartialSupport, Some("origin-c"), 3),
        ];
        let rule = SupportRuleV1::AnyOf {
            alternative_support_sets: vec![vec!["a".to_owned()], vec!["b".to_owned()]],
        };
        assert_eq!(
            evaluate_support_sufficiency(&rule, &evidence),
            Ok(SupportSufficiencyV1::Supported)
        );
    }

    #[test]
    fn all_of_requires_every_required_evidence_item() {
        let evidence = vec![
            item("a", SupportEvidenceStateV1::DirectSupport, Some("origin-a"), 1),
            item("b", SupportEvidenceStateV1::Stale, Some("origin-b"), 2),
        ];
        let rule = SupportRuleV1::AllOf {
            required_evidence_ids: vec!["a".to_owned(), "b".to_owned()],
        };
        assert_eq!(
            evaluate_support_sufficiency(&rule, &evidence),
            Ok(SupportSufficiencyV1::InsufficientEvidence)
        );
    }

    #[test]
    fn common_cause_groups_prevent_k_of_n_inflation() {
        let evidence = vec![
            item("a", SupportEvidenceStateV1::DirectSupport, Some("shared-origin"), 1),
            item("b", SupportEvidenceStateV1::DirectSupport, Some("shared-origin"), 2),
            item("c", SupportEvidenceStateV1::DirectSupport, Some("distinct-origin"), 3),
        ];
        let rule = SupportRuleV1::AtLeastIndependent {
            candidate_evidence_ids: vec!["a".to_owned(), "b".to_owned(), "c".to_owned()],
            minimum: 2,
        };
        assert_eq!(
            evaluate_support_sufficiency(&rule, &evidence),
            Ok(SupportSufficiencyV1::Supported)
        );

        let rule = SupportRuleV1::AtLeastIndependent {
            candidate_evidence_ids: vec!["a".to_owned(), "b".to_owned()],
            minimum: 2,
        };
        assert_eq!(
            evaluate_support_sufficiency(&rule, &evidence),
            Ok(SupportSufficiencyV1::InsufficientEvidence)
        );
    }

    #[test]
    fn missing_common_cause_group_is_not_counted_as_independent() {
        let evidence = vec![
            item("a", SupportEvidenceStateV1::DirectSupport, None, 1),
            item("b", SupportEvidenceStateV1::DirectSupport, Some("origin-b"), 2),
        ];
        let rule = SupportRuleV1::AtLeastIndependent {
            candidate_evidence_ids: vec!["a".to_owned(), "b".to_owned()],
            minimum: 2,
        };
        assert_eq!(
            evaluate_support_sufficiency(&rule, &evidence),
            Err(SupportSufficiencyErrorV1::MissingIndependenceGroup)
        );
    }

    #[test]
    fn contradictory_candidate_requires_review_even_if_another_supports() {
        let evidence = vec![
            item("a", SupportEvidenceStateV1::DirectSupport, Some("origin-a"), 1),
            item("b", SupportEvidenceStateV1::Contradicted, Some("origin-b"), 2),
        ];
        let rule = SupportRuleV1::AnyOf {
            alternative_support_sets: vec![vec!["a".to_owned()], vec!["b".to_owned()]],
        };
        assert_eq!(
            evaluate_support_sufficiency(&rule, &evidence),
            Ok(SupportSufficiencyV1::ConflictReviewRequired)
        );
    }

    #[test]
    fn exhaustive_k_of_n_matches_distinct_group_oracle() {
        let groups = ["source-a", "source-a", "source-b", "source-c"];
        for mask in 0_u8..16 {
            let evidence = (0..4)
                .map(|index| {
                    let state = if mask & (1 << index) != 0 {
                        SupportEvidenceStateV1::DirectSupport
                    } else {
                        SupportEvidenceStateV1::Stale
                    };
                    item(
                        &format!("e{index}"),
                        state,
                        Some(groups[index]),
                        index as u8 + 1,
                    )
                })
                .collect::<Vec<_>>();
            let direct_groups = (0..4)
                .filter(|index| mask & (1 << index) != 0)
                .map(|index| groups[index])
                .collect::<BTreeSet<_>>();

            for minimum in 1..=4 {
                let rule = SupportRuleV1::AtLeastIndependent {
                    candidate_evidence_ids: (0..4).map(|index| format!("e{index}")).collect(),
                    minimum,
                };
                let expected = if direct_groups.len() >= minimum {
                    SupportSufficiencyV1::Supported
                } else {
                    SupportSufficiencyV1::InsufficientEvidence
                };
                assert_eq!(
                    evaluate_support_sufficiency(&rule, &evidence),
                    Ok(expected),
                    "support mask {mask}, minimum {minimum}"
                );
            }
        }
    }

    #[test]
    fn exhaustive_all_of_and_any_of_match_independent_four_candidate_oracle() {
        let states = [
            SupportEvidenceStateV1::DirectSupport,
            SupportEvidenceStateV1::PartialSupport,
            SupportEvidenceStateV1::Contradicted,
            SupportEvidenceStateV1::Stale,
            SupportEvidenceStateV1::NotAssessable,
        ];
        let ids = (0..4).map(|index| format!("e{index}")).collect::<Vec<_>>();
        let all_of = SupportRuleV1::AllOf {
            required_evidence_ids: ids.clone(),
        };
        let any_of_singletons = SupportRuleV1::AnyOf {
            alternative_support_sets: ids
                .iter()
                .map(|id| vec![id.clone()])
                .collect(),
        };
        let any_of_pairs = SupportRuleV1::AnyOf {
            alternative_support_sets: vec![
                vec![ids[0].clone(), ids[1].clone()],
                vec![ids[2].clone(), ids[3].clone()],
            ],
        };

        // 5^4 = 625 possible evidence-state assignments.
        for mut mask in 0_usize..625 {
            let mut selected = Vec::with_capacity(4);
            for index in 0..4 {
                selected.push(states[mask % states.len()]);
                mask /= states.len();
            }
            let evidence = selected
                .iter()
                .enumerate()
                .map(|(index, state)| {
                    item(
                        &ids[index],
                        *state,
                        Some(&format!("origin-{index}")),
                        index as u8 + 1,
                    )
                })
                .collect::<Vec<_>>();

            let any_contradiction = selected
                .iter()
                .any(|state| *state == SupportEvidenceStateV1::Contradicted);
            let all_direct = selected
                .iter()
                .all(|state| *state == SupportEvidenceStateV1::DirectSupport);
            let any_direct = selected
                .iter()
                .any(|state| *state == SupportEvidenceStateV1::DirectSupport);
            let pair_01 = selected[0] == SupportEvidenceStateV1::DirectSupport
                && selected[1] == SupportEvidenceStateV1::DirectSupport;
            let pair_23 = selected[2] == SupportEvidenceStateV1::DirectSupport
                && selected[3] == SupportEvidenceStateV1::DirectSupport;

            let expected_all = if any_contradiction {
                SupportSufficiencyV1::ConflictReviewRequired
            } else if all_direct {
                SupportSufficiencyV1::Supported
            } else {
                SupportSufficiencyV1::InsufficientEvidence
            };
            let expected_any_single = if any_contradiction {
                SupportSufficiencyV1::ConflictReviewRequired
            } else if any_direct {
                SupportSufficiencyV1::Supported
            } else {
                SupportSufficiencyV1::InsufficientEvidence
            };
            let expected_any_pair = if any_contradiction {
                SupportSufficiencyV1::ConflictReviewRequired
            } else if pair_01 || pair_23 {
                SupportSufficiencyV1::Supported
            } else {
                SupportSufficiencyV1::InsufficientEvidence
            };

            assert_eq!(
                evaluate_support_sufficiency(&all_of, &evidence),
                Ok(expected_all),
                "all-of mask {mask}"
            );
            assert_eq!(
                evaluate_support_sufficiency(&any_of_singletons, &evidence),
                Ok(expected_any_single),
                "any-of singleton mask {mask}"
            );
            assert_eq!(
                evaluate_support_sufficiency(&any_of_pairs, &evidence),
                Ok(expected_any_pair),
                "any-of pair mask {mask}"
            );
        }
    }

}
