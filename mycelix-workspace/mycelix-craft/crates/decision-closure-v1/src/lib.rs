//! Conclusion-relative, material-element evidence closure.
//!
//! This crate evaluates a finite, authority-reviewed material-element mapping
//! against host-supplied claim-support result references. It does not define
//! the domain's legal/clinical/technical standard, authenticate external
//! authorities, or decide that a mapping is complete.
//!
//! Closure is stricter than atomic claim support: every required element,
//! exception and prerequisite must be covered for the exact conclusion.

#![forbid(unsafe_code)]

use std::collections::{HashMap, HashSet};

/// Current decision-closure contract schema version.
pub const DECISION_CLOSURE_SCHEMA_VERSION: u16 = 1;

/// Local authority reference. The host authenticates issuer, scope and revocation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LocalAuthorityRefV1 {
    /// Local authority policy identifier.
    pub policy_id: String,
    /// Adopted policy revision.
    pub revision: u64,
    /// Exact policy digest.
    pub policy_digest: [u8; 32],
    /// Local decision/delegation identifier.
    pub decision_id: String,
    /// Exact local decision digest.
    pub decision_digest: [u8; 32],
}

/// Exact regulated, safety, administrative or other consequential conclusion.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DecisionContextV1 {
    /// Stable decision identifier.
    pub decision_id: String,
    /// Exact conclusion text/structured-content digest.
    pub conclusion_digest: [u8; 32],
    /// Domain under whose rules the conclusion is evaluated.
    pub domain_id: String,
    /// Jurisdiction in which the conclusion is asserted.
    pub jurisdiction_id: String,
    /// Time at which the conclusion is evaluated.
    pub decision_time: u64,
    /// Authorized evidence corpus identifier.
    pub authorized_corpus_id: String,
    /// Exact authorized corpus snapshot digest.
    pub authorized_corpus_digest: [u8; 32],
    /// Domain rule/standard identifier.
    pub standard_id: String,
    /// Domain rule/standard revision.
    pub standard_revision: u64,
    /// Exact standard digest.
    pub standard_digest: [u8; 32],
    /// Reviewer roles required to approve a material-element mapping.
    pub required_mapping_roles: Vec<MappingReviewerRoleV1>,
    /// Minimum distinct reviewers required for each required role.
    /// Set to 2 or more when independent mapping agreement is part of qualification.
    pub minimum_reviewers_per_role: u16,
}

/// Why a material element is part of the conclusion's element structure.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum MaterialElementKindV1 {
    /// The governing rule, policy or standard applies.
    RuleApplicability,
    /// Subject/population is within scope.
    SubjectScope,
    /// A required action or control was implemented.
    Implementation,
    /// The actor or assessor had the required authority.
    ActorAuthority,
    /// The action occurred within the required time.
    Timing,
    /// Exception, contraindication or condition was handled.
    ExceptionHandling,
    /// Monitoring, review or observation occurred.
    Monitoring,
    /// Required records or documentation exist.
    Documentation,
    /// Corrective action or remediation occurred.
    CorrectiveAction,
    /// A domain-specific material element not covered above.
    DomainSpecific,
}

/// Applicability of a material element.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ElementApplicabilityV1 {
    /// The element applies and must be covered by evidence.
    Applicable,
    /// A domain/local authority has determined this element does not apply.
    NotApplicable {
        /// Exact decision that established non-applicability.
        decision_id: String,
        /// Digest of the non-applicability decision.
        decision_digest: [u8; 32],
        /// Local authority who issued the decision.
        authority: LocalAuthorityRefV1,
    },
    /// Applicability has not been resolved; this never counts as covered.
    Unresolved,
}

/// The claim-support relation supplied by the external Claim Support verifier.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDispositionV1 {
    /// Exact claim has current, independently reviewed direct support.
    DirectSupport,
    /// Some, but not all, of the claim is supported.
    PartialSupport,
    /// The evidence contradicts the claim.
    Contradiction,
    /// More context or qualified review is required.
    HumanReviewRequired,
    /// Evidence is unavailable or insufficient.
    InsufficientEvidence,
}

/// A digest-bound reference to a claim-support result.
///
/// This is an input boundary. The host must authenticate that the referenced
/// result really came from Claim Support V1 and is current before evaluation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ClaimSupportResultRefV1 {
    /// Stable support-result identifier.
    pub result_id: String,
    /// Digest of the full support result, including role reviews.
    pub result_digest: [u8; 32],
    /// Exact claim identifier.
    pub claim_id: String,
    /// Exact claim digest.
    pub claim_digest: [u8; 32],
    /// Exact proposal revision digest containing the claim.
    pub proposal_digest: [u8; 32],
    /// Domain of the support assessment.
    pub domain_id: String,
    /// Jurisdiction of the support assessment.
    pub jurisdiction_id: String,
    /// Exact authorized corpus snapshot used.
    pub corpus_digest: [u8; 32],
    /// Support disposition.
    pub disposition: SupportDispositionV1,
    /// Time at which the support result was evaluated.
    pub evaluated_at: u64,
    /// Optional expiry of the support result.
    pub valid_until: Option<u64>,
}

/// Why an element must be supported before a conclusion can be closed.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MaterialElementV1 {
    /// Stable material-element identifier within this mapping.
    pub element_id: String,
    /// Human-readable statement of the material condition.
    pub statement: String,
    /// Element kind.
    pub kind: MaterialElementKindV1,
    /// Applies, has been authoritatively excluded, or remains unresolved.
    pub applicability: ElementApplicabilityV1,
    /// Prerequisite material elements that must themselves be satisfied.
    pub depends_on: Vec<String>,
    /// Minimum number of distinct direct-support result references required.
    pub minimum_support_results: u16,
    /// Claim-support results proposed as evidence for this element.
    pub support_results: Vec<ClaimSupportResultRefV1>,
}

/// One reviewer role authorized for material-element mapping approval.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum MappingReviewerRoleV1 {
    /// Domain expert qualified to identify and interpret elements.
    DomainExpert,
    /// Legal, statutory or regulatory authority where required.
    LegalAuthority,
    /// Clinical authority where required.
    ClinicalAuthority,
    /// Safety authority where required.
    SafetyAuthority,
    /// Source/evidence governance authority.
    EvidenceAuthority,
}

/// Decision on an exact element mapping.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MappingReviewDecisionV1 {
    /// Reviewer approves the exact mapping digest.
    Approve,
    /// Reviewer requests changes; this mapping cannot close a decision.
    RequestChanges,
    /// Reviewer rejects this mapping revision.
    Reject,
}

/// Review of one exact material-element mapping.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MappingReviewV1 {
    /// Stable review identifier.
    pub review_id: String,
    /// Mapping identifier.
    pub mapping_id: String,
    /// Exact mapping digest.
    pub mapping_digest: [u8; 32],
    /// Independent reviewer.
    pub reviewer_id: String,
    /// Reviewer role.
    pub role: MappingReviewerRoleV1,
    /// Local authority for this review.
    pub authority: LocalAuthorityRefV1,
    /// Decision.
    pub decision: MappingReviewDecisionV1,
    /// Review timestamp.
    pub reviewed_at: u64,
    /// Optional expiry.
    pub valid_until: Option<u64>,
    /// Optional effective withdrawal time.
    pub withdrawn_at: Option<u64>,
}

/// One defensible element mapping for a decision conclusion.
///
/// Multiple mappings are allowed because element decomposition can involve
/// expert judgment. A robustly closed verdict must be closed under every
/// currently accepted mapping supplied in the packet.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MaterialElementMappingV1 {
    /// Stable mapping identifier.
    pub mapping_id: String,
    /// Revision number.
    pub revision: u64,
    /// Previous mapping digest, if amended.
    pub previous_mapping_digest: Option<[u8; 32]>,
    /// Digest of the exact canonical mapping bytes, computed by the host.
    pub mapping_digest: [u8; 32],
    /// Decision conclusion this mapping applies to.
    pub decision_id: String,
    /// Exact conclusion digest.
    pub conclusion_digest: [u8; 32],
    /// Exact domain standard digest used for the mapping.
    pub standard_digest: [u8; 32],
    /// Material elements, dependencies and evidence references.
    pub elements: Vec<MaterialElementV1>,
    /// Reviews that bind to this exact mapping digest.
    pub reviews: Vec<MappingReviewV1>,
}

/// A packet for evaluating one conclusion under one or more accepted mappings.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DecisionClosurePacketV1 {
    /// Schema version.
    pub schema_version: u16,
    /// Exact decision/conclusion context.
    pub decision: DecisionContextV1,
    /// Multiple defensible mappings; all must be closed for robust closure.
    pub mappings: Vec<MaterialElementMappingV1>,
}

/// Structural or reference integrity errors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecisionClosureErrorV1 {
    /// Unsupported schema.
    UnsupportedSchema,
    /// Required identifier is empty or padded.
    EmptyField,
    /// Required digest is missing.
    MissingDigest,
    /// Required revision or count is zero.
    InvalidRevision,
    /// Identifier or role is duplicated.
    DuplicateIdentifier,
    /// A material element references an unknown dependency.
    UnknownDependency,
    /// A dependency cycle exists.
    DependencyCycle,
    /// Mapping references a different decision/conclusion/standard.
    DecisionReferenceMismatch,
    /// Evidence support reference has the wrong domain, jurisdiction or corpus.
    EvidenceScopeMismatch,
    /// Element has a malformed applicability decision.
    InvalidApplicabilityDecision,
    /// An applicable element has a zero support threshold.
    ZeroSupportThreshold,
    /// A mapping has no material elements.
    EmptyMapping,
    /// A review does not reference its exact mapping.
    ReviewReferenceMismatch,
    /// A reviewer has no valid local authority reference.
    InvalidAuthority,
    /// Review timing is invalid.
    InvalidReviewTime,
    /// A same-reviewer duplicate or conflicting decision exists.
    DuplicateReview,
}

/// Status for one material element under one mapping.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ElementClosureStatusV1 {
    /// Element has current, in-scope direct support and satisfied prerequisites.
    Satisfied,
    /// Element is explicitly ruled not applicable by a local authority decision.
    NotApplicable,
    /// No sufficient direct-support evidence is available.
    MissingEvidence,
    /// Evidence exists but does not directly support the element.
    Unsupported,
    /// Evidence contradicts the required element.
    Contradicted,
    /// A prerequisite element is not satisfied.
    PrerequisiteUnmet,
    /// Applicability or required support needs review.
    HumanReviewRequired,
}

/// Coverage result for one material element.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ElementClosureResultV1 {
    /// Element identifier.
    pub element_id: String,
    /// Element status.
    pub status: ElementClosureStatusV1,
    /// Current support-result IDs counted toward coverage.
    pub counted_support_result_ids: Vec<String>,
    /// Support-result IDs rejected due to stale or invalid context.
    pub unusable_support_result_ids: Vec<String>,
}

/// Overall closure disposition.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecisionClosureDispositionV1 {
    /// Every required element is satisfied under every approved mapping.
    Closed,
    /// At least one material element lacks sufficient evidence.
    OpenMissingElements,
    /// At least one element is contradicted.
    BlockedByContradiction,
    /// An element's prerequisites are not satisfied.
    BlockedByPrerequisite,
    /// Different accepted mappings produce different closure verdicts.
    MappingSensitive,
    /// One or more required element mappings lack valid approval.
    MappingReviewRequired,
    /// No material-element mapping is available.
    InsufficientMapping,
}

/// Closure evaluation for one mapping.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MappingClosureEvaluationV1 {
    /// Mapping identifier and digest.
    pub mapping_id: String,
    pub mapping_digest: [u8; 32],
    /// Mapping approved for this exact decision and time.
    pub mapping_approved: bool,
    /// Element-level results.
    pub elements: Vec<ElementClosureResultV1>,
    /// Closure result for this mapping.
    pub disposition: DecisionClosureDispositionV1,
}

/// Full conclusion-relative closure evaluation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DecisionClosureEvaluationV1 {
    /// Exact decision/conclusion identifiers.
    pub decision_id: String,
    pub conclusion_digest: [u8; 32],
    /// Mapping-level outcomes.
    pub mappings: Vec<MappingClosureEvaluationV1>,
    /// Aggregate result across every supplied mapping.
    pub disposition: DecisionClosureDispositionV1,
}

/// Validate authority reference. Authenticity/freshness is host responsibility.
fn validate_authority(authority: &LocalAuthorityRefV1) -> Result<(), DecisionClosureErrorV1> {
    require_non_empty(&authority.policy_id)?;
    require_non_empty(&authority.decision_id)?;
    if authority.revision == 0 {
        return Err(DecisionClosureErrorV1::InvalidRevision);
    }
    require_digest(&authority.policy_digest)?;
    require_digest(&authority.decision_digest)
}

/// Validate a closure packet before evaluation.
pub fn validate_packet(packet: &DecisionClosurePacketV1) -> Result<(), DecisionClosureErrorV1> {
    if packet.schema_version != DECISION_CLOSURE_SCHEMA_VERSION {
        return Err(DecisionClosureErrorV1::UnsupportedSchema);
    }

    let decision = &packet.decision;
    require_non_empty(&decision.decision_id)?;
    require_digest(&decision.conclusion_digest)?;
    require_non_empty(&decision.domain_id)?;
    require_non_empty(&decision.jurisdiction_id)?;
    require_non_empty(&decision.authorized_corpus_id)?;
    require_digest(&decision.authorized_corpus_digest)?;
    require_non_empty(&decision.standard_id)?;
    require_digest(&decision.standard_digest)?;
    if decision.standard_revision == 0
        || decision.required_mapping_roles.is_empty()
        || decision.minimum_reviewers_per_role == 0
    {
        return Err(DecisionClosureErrorV1::InvalidRevision);
    }
    let mut required_roles = HashSet::new();
    for role in &decision.required_mapping_roles {
        if !required_roles.insert(*role) {
            return Err(DecisionClosureErrorV1::DuplicateIdentifier);
        }
    }

    if packet.mappings.is_empty() {
        return Ok(());
    }

    let mut mapping_ids = HashSet::new();
    for mapping in &packet.mappings {
        require_non_empty(&mapping.mapping_id)?;
        require_digest(&mapping.mapping_digest)?;
        require_digest(&mapping.conclusion_digest)?;
        require_digest(&mapping.standard_digest)?;
        if mapping.revision == 0 {
            return Err(DecisionClosureErrorV1::InvalidRevision);
        }
        if (mapping.revision == 1 && mapping.previous_mapping_digest.is_some())
            || (mapping.revision > 1
                && mapping.previous_mapping_digest
                    .map(|digest| digest.iter().all(|byte| *byte == 0))
                    .unwrap_or(true))
        {
            return Err(DecisionClosureErrorV1::InvalidRevision);
        }
        if mapping.decision_id != decision.decision_id
            || mapping.conclusion_digest != decision.conclusion_digest
            || mapping.standard_digest != decision.standard_digest
        {
            return Err(DecisionClosureErrorV1::DecisionReferenceMismatch);
        }
        if !mapping_ids.insert(mapping.mapping_id.as_str()) {
            return Err(DecisionClosureErrorV1::DuplicateIdentifier);
        }
        if mapping.elements.is_empty() {
            return Err(DecisionClosureErrorV1::EmptyMapping);
        }

        let mut elements = HashMap::new();
        for element in &mapping.elements {
            require_non_empty(&element.element_id)?;
            require_non_empty(&element.statement)?;
            match &element.applicability {
                ElementApplicabilityV1::Applicable => {
                    if element.minimum_support_results == 0 {
                        return Err(DecisionClosureErrorV1::ZeroSupportThreshold);
                    }
                }
                ElementApplicabilityV1::Unresolved => {
                    if element.minimum_support_results == 0 {
                        return Err(DecisionClosureErrorV1::ZeroSupportThreshold);
                    }
                }
                ElementApplicabilityV1::NotApplicable {
                    decision_id,
                    decision_digest,
                    authority,
                } => {
                    require_non_empty(decision_id)?;
                    require_digest(decision_digest)?;
                    validate_authority(authority)?;
                }
            }
            if elements.insert(element.element_id.as_str(), element).is_some() {
                return Err(DecisionClosureErrorV1::DuplicateIdentifier);
            }

            let mut result_ids = HashSet::new();
            for support in &element.support_results {
                require_non_empty(&support.result_id)?;
                require_digest(&support.result_digest)?;
                require_non_empty(&support.claim_id)?;
                require_digest(&support.claim_digest)?;
                require_digest(&support.proposal_digest)?;
                require_non_empty(&support.domain_id)?;
                require_non_empty(&support.jurisdiction_id)?;
                require_digest(&support.corpus_digest)?;
                if !result_ids.insert(support.result_id.as_str()) {
                    return Err(DecisionClosureErrorV1::DuplicateIdentifier);
                }
                if support.domain_id != decision.domain_id
                    || support.jurisdiction_id != decision.jurisdiction_id
                    || support.corpus_digest != decision.authorized_corpus_digest
                {
                    return Err(DecisionClosureErrorV1::EvidenceScopeMismatch);
                }
                if support.valid_until.map(|expiry| expiry < support.evaluated_at).unwrap_or(false) {
                    return Err(DecisionClosureErrorV1::InvalidReviewTime);
                }
            }
        }
        for element in &mapping.elements {
            let mut seen = HashSet::new();
            for dependency in &element.depends_on {
                if dependency == &element.element_id || !elements.contains_key(dependency.as_str()) {
                    return Err(DecisionClosureErrorV1::UnknownDependency);
                }
                if !seen.insert(dependency.as_str()) {
                    return Err(DecisionClosureErrorV1::DuplicateIdentifier);
                }
            }
        }
        validate_dependency_graph(&mapping.elements)?;

        let mut review_ids = HashSet::new();
        let mut role_reviewers = HashSet::new();
        for review in &mapping.reviews {
            require_non_empty(&review.review_id)?;
            require_non_empty(&review.mapping_id)?;
            require_non_empty(&review.reviewer_id)?;
            validate_authority(&review.authority)?;
            require_digest(&review.mapping_digest)?;
            if review.mapping_id != mapping.mapping_id || review.mapping_digest != mapping.mapping_digest {
                return Err(DecisionClosureErrorV1::ReviewReferenceMismatch);
            }
            if !review_ids.insert(review.review_id.as_str())
                || !role_reviewers.insert((review.role, review.reviewer_id.as_str()))
            {
                return Err(DecisionClosureErrorV1::DuplicateReview);
            }
            if review.reviewed_at > decision.decision_time
                || review.valid_until.map(|expiry| expiry < review.reviewed_at).unwrap_or(false)
                || review.withdrawn_at.map(|time| time < review.reviewed_at).unwrap_or(false)
            {
                return Err(DecisionClosureErrorV1::InvalidReviewTime);
            }
        }
    }

    Ok(())
}

fn validate_dependency_graph(elements: &[MaterialElementV1]) -> Result<(), DecisionClosureErrorV1> {
    fn visit(
        id: &str,
        graph: &HashMap<&str, &MaterialElementV1>,
        visiting: &mut HashSet<String>,
        visited: &mut HashSet<String>,
    ) -> Result<(), DecisionClosureErrorV1> {
        if visited.contains(id) {
            return Ok(());
        }
        if !visiting.insert(id.to_owned()) {
            return Err(DecisionClosureErrorV1::DependencyCycle);
        }
        let element = graph.get(id).ok_or(DecisionClosureErrorV1::UnknownDependency)?;
        for dependency in &element.depends_on {
            visit(dependency, graph, visiting, visited)?;
        }
        visiting.remove(id);
        visited.insert(id.to_owned());
        Ok(())
    }
    let graph = elements.iter().map(|element| (element.element_id.as_str(), element)).collect::<HashMap<_, _>>();
    let mut visiting = HashSet::new();
    let mut visited = HashSet::new();
    for id in graph.keys() {
        visit(id, &graph, &mut visiting, &mut visited)?;
    }
    Ok(())
}

fn mapping_approved(
    decision: &DecisionContextV1,
    mapping: &MaterialElementMappingV1,
    now: u64,
) -> bool {
    let current = mapping.reviews.iter().filter(|review| {
        review.mapping_digest == mapping.mapping_digest
            && review.reviewed_at <= now
            && review.valid_until.map(|expiry| now < expiry).unwrap_or(true)
            && review.withdrawn_at.map(|time| now < time).unwrap_or(true)
    }).collect::<Vec<_>>();

    if current.iter().any(|review| review.decision != MappingReviewDecisionV1::Approve) {
        return false;
    }

    decision.required_mapping_roles.iter().all(|role| {
        current
            .iter()
            .filter(|review| review.role == *role && review.decision == MappingReviewDecisionV1::Approve)
            .map(|review| review.reviewer_id.as_str())
            .collect::<HashSet<_>>()
            .len()
            >= decision.minimum_reviewers_per_role as usize
    })
}

/// Evaluate one approved mapping against exact support-result references.
fn evaluate_mapping(
    decision: &DecisionContextV1,
    mapping: &MaterialElementMappingV1,
    now: u64,
) -> MappingClosureEvaluationV1 {
    let approved = mapping_approved(decision, mapping, now);
    if !approved {
        return MappingClosureEvaluationV1 {
            mapping_id: mapping.mapping_id.clone(),
            mapping_digest: mapping.mapping_digest,
            mapping_approved: false,
            elements: Vec::new(),
            disposition: DecisionClosureDispositionV1::MappingReviewRequired,
        };
    }

    let mut results = HashMap::new();
    let mut status = HashMap::new();
    for element in &mapping.elements {
        let result = match &element.applicability {
            ElementApplicabilityV1::NotApplicable { .. } => ElementClosureStatusV1::NotApplicable,
            ElementApplicabilityV1::Unresolved => ElementClosureStatusV1::HumanReviewRequired,
            ElementApplicabilityV1::Applicable => {
                let mut counted = Vec::new();
                let mut unusable = Vec::new();
                let mut contradicted = false;
                let mut needs_review = false;
                for support in &element.support_results {
                    let current = support.evaluated_at <= now
                        && support.evaluated_at <= decision.decision_time
                        && support.valid_until.map(|expiry| now < expiry).unwrap_or(true);
                    if !current {
                        unusable.push(support.result_id.clone());
                        continue;
                    }
                    match support.disposition {
                        SupportDispositionV1::DirectSupport => counted.push(support.result_id.clone()),
                        SupportDispositionV1::Contradiction => contradicted = true,
                        SupportDispositionV1::PartialSupport
                        | SupportDispositionV1::HumanReviewRequired
                        | SupportDispositionV1::InsufficientEvidence => needs_review = true,
                    }
                }
                if contradicted {
                    ElementClosureStatusV1::Contradicted
                } else if counted.len() >= element.minimum_support_results as usize {
                    results.insert(element.element_id.as_str(), (counted, unusable));
                    ElementClosureStatusV1::Satisfied
                } else if needs_review || !unusable.is_empty() {
                    results.insert(element.element_id.as_str(), (counted, unusable));
                    ElementClosureStatusV1::HumanReviewRequired
                } else {
                    results.insert(element.element_id.as_str(), (counted, unusable));
                    ElementClosureStatusV1::MissingEvidence
                }
            }
        };
        status.insert(element.element_id.as_str(), result);
    }

    // A dependency can only be treated as satisfied if every prerequisite is
    // itself directly satisfied or authoritatively excluded as not applicable.
    let mut changed = true;
    while changed {
        changed = false;
        for element in &mapping.elements {
            if status.get(element.element_id.as_str()) == Some(&ElementClosureStatusV1::Satisfied) {
                if element.depends_on.iter().any(|dependency| {
                    !matches!(
                        status.get(dependency.as_str()),
                        Some(ElementClosureStatusV1::Satisfied | ElementClosureStatusV1::NotApplicable)
                    )
                }) {
                    status.insert(element.element_id.as_str(), ElementClosureStatusV1::PrerequisiteUnmet);
                    changed = true;
                }
            }
        }
    }

    let elements = mapping.elements.iter().map(|element| {
        let (counted, unusable) = results.remove(element.element_id.as_str()).unwrap_or_default();
        ElementClosureResultV1 {
            element_id: element.element_id.clone(),
            status: *status.get(element.element_id.as_str()).unwrap_or(&ElementClosureStatusV1::HumanReviewRequired),
            counted_support_result_ids: counted,
            unusable_support_result_ids: unusable,
        }
    }).collect::<Vec<_>>();

    let disposition = if elements.iter().any(|element| element.status == ElementClosureStatusV1::Contradicted) {
        DecisionClosureDispositionV1::BlockedByContradiction
    } else if elements.iter().any(|element| element.status == ElementClosureStatusV1::PrerequisiteUnmet) {
        DecisionClosureDispositionV1::BlockedByPrerequisite
    } else if elements.iter().any(|element| element.status == ElementClosureStatusV1::HumanReviewRequired) {
        DecisionClosureDispositionV1::MappingReviewRequired
    } else if elements.iter().any(|element| element.status == ElementClosureStatusV1::MissingEvidence
        || element.status == ElementClosureStatusV1::Unsupported)
    {
        DecisionClosureDispositionV1::OpenMissingElements
    } else {
        DecisionClosureDispositionV1::Closed
    };

    MappingClosureEvaluationV1 {
        mapping_id: mapping.mapping_id.clone(),
        mapping_digest: mapping.mapping_digest,
        mapping_approved: true,
        elements,
        disposition,
    }
}

/// Evaluate conclusion-relative closure across all accepted material-element mappings.
///
/// A robustly Closed result requires every currently supplied approved mapping to
/// be closed. If mappings disagree, return MappingSensitive rather than choosing
/// the most favorable decomposition.
pub fn evaluate_decision_closure(
    packet: &DecisionClosurePacketV1,
    now: u64,
) -> Result<DecisionClosureEvaluationV1, DecisionClosureErrorV1> {
    validate_packet(packet)?;
    if packet.mappings.is_empty() {
        return Ok(DecisionClosureEvaluationV1 {
            decision_id: packet.decision.decision_id.clone(),
            conclusion_digest: packet.decision.conclusion_digest,
            mappings: Vec::new(),
            disposition: DecisionClosureDispositionV1::InsufficientMapping,
        });
    }

    let evaluations = packet.mappings.iter().map(|mapping| {
        evaluate_mapping(&packet.decision, mapping, now)
    }).collect::<Vec<_>>();

    let disposition = if evaluations.iter().any(|item| item.disposition == DecisionClosureDispositionV1::BlockedByContradiction) {
        DecisionClosureDispositionV1::BlockedByContradiction
    } else if evaluations.iter().any(|item| item.disposition == DecisionClosureDispositionV1::BlockedByPrerequisite) {
        DecisionClosureDispositionV1::BlockedByPrerequisite
    } else if evaluations.iter().any(|item| item.disposition == DecisionClosureDispositionV1::MappingReviewRequired) {
        DecisionClosureDispositionV1::MappingReviewRequired
    } else if evaluations.iter().all(|item| item.disposition == DecisionClosureDispositionV1::Closed) {
        DecisionClosureDispositionV1::Closed
    } else if evaluations.iter().any(|item| item.disposition == DecisionClosureDispositionV1::Closed)
        && evaluations.iter().any(|item| item.disposition != DecisionClosureDispositionV1::Closed)
    {
        DecisionClosureDispositionV1::MappingSensitive
    } else {
        DecisionClosureDispositionV1::OpenMissingElements
    };

    Ok(DecisionClosureEvaluationV1 {
        decision_id: packet.decision.decision_id.clone(),
        conclusion_digest: packet.decision.conclusion_digest,
        mappings: evaluations,
        disposition,
    })
}

fn require_non_empty(value: &str) -> Result<(), DecisionClosureErrorV1> {
    if !value.is_empty() && value.trim() == value {
        Ok(())
    } else {
        Err(DecisionClosureErrorV1::EmptyField)
    }
}

fn require_digest(value: &[u8; 32]) -> Result<(), DecisionClosureErrorV1> {
    if value.iter().all(|byte| *byte == 0) {
        Err(DecisionClosureErrorV1::MissingDigest)
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
            decision_id: format!("authority-decision-{id}"),
            decision_digest: [2; 32],
        }
    }

    fn decision() -> DecisionContextV1 {
        DecisionContextV1 {
            decision_id: "water-service-compliance".to_owned(),
            conclusion_digest: [3; 32],
            domain_id: "water-service-assurance-v1".to_owned(),
            jurisdiction_id: "ZA-GP".to_owned(),
            decision_time: NOW,
            authorized_corpus_id: "water-authority-corpus".to_owned(),
            authorized_corpus_digest: [4; 32],
            standard_id: "water-service-standard".to_owned(),
            standard_revision: 1,
            standard_digest: [5; 32],
            required_mapping_roles: vec![MappingReviewerRoleV1::DomainExpert],
            minimum_reviewers_per_role: 2,
        }
    }

    fn support(id: &str, disposition: SupportDispositionV1) -> ClaimSupportResultRefV1 {
        ClaimSupportResultRefV1 {
            result_id: id.to_owned(),
            result_digest: [6; 32],
            claim_id: format!("claim-{id}"),
            claim_digest: [7; 32],
            proposal_digest: [8; 32],
            domain_id: "water-service-assurance-v1".to_owned(),
            jurisdiction_id: "ZA-GP".to_owned(),
            corpus_digest: [4; 32],
            disposition,
            evaluated_at: 10,
            valid_until: Some(200),
        }
    }

    fn element(id: &str, dependencies: Vec<String>, supports: Vec<ClaimSupportResultRefV1>) -> MaterialElementV1 {
        MaterialElementV1 {
            element_id: id.to_owned(),
            statement: format!("Required material element {id}"),
            kind: MaterialElementKindV1::DomainSpecific,
            applicability: ElementApplicabilityV1::Applicable,
            depends_on: dependencies,
            minimum_support_results: 1,
            support_results: supports,
        }
    }

    fn review(mapping_id: &str, digest: [u8; 32]) -> MappingReviewV1 {
        MappingReviewV1 {
            review_id: format!("review-{mapping_id}"),
            mapping_id: mapping_id.to_owned(),
            mapping_digest: digest,
            reviewer_id: "independent-domain-reviewer".to_owned(),
            role: MappingReviewerRoleV1::DomainExpert,
            authority: authority("domain"),
            decision: MappingReviewDecisionV1::Approve,
            reviewed_at: 20,
            valid_until: Some(180),
            withdrawn_at: None,
        }
    }

    fn mapping(id: &str, elements: Vec<MaterialElementV1>) -> MaterialElementMappingV1 {
        let digest = [9; 32];
        MaterialElementMappingV1 {
            mapping_id: id.to_owned(),
            revision: 1,
            previous_mapping_digest: None,
            mapping_digest: digest,
            decision_id: "water-service-compliance".to_owned(),
            conclusion_digest: [3; 32],
            standard_digest: [5; 32],
            elements,
            reviews: vec![
                review(id, digest),
                MappingReviewV1 {
                    review_id: format!("review-{id}-second"),
                    mapping_id: id.to_owned(),
                    mapping_digest: digest,
                    reviewer_id: "independent-domain-reviewer-2".to_owned(),
                    role: MappingReviewerRoleV1::DomainExpert,
                    authority: authority("domain-2"),
                    decision: MappingReviewDecisionV1::Approve,
                    reviewed_at: 21,
                    valid_until: Some(180),
                    withdrawn_at: None,
                },
            ],
        }
    }

    fn packet(mappings: Vec<MaterialElementMappingV1>) -> DecisionClosurePacketV1 {
        DecisionClosurePacketV1 {
            schema_version: DECISION_CLOSURE_SCHEMA_VERSION,
            decision: decision(),
            mappings,
        }
    }

    #[test]
    fn supported_atomic_claims_do_not_close_an_incomplete_decision() {
        let mapping = mapping(
            "mapping-a",
            vec![
                element("rule-exists", Vec::new(), vec![support("rule", SupportDispositionV1::DirectSupport)]),
                element("implementation", vec!["rule-exists".to_owned()], Vec::new()),
                element("timing", Vec::new(), Vec::new()),
                element("authority", Vec::new(), Vec::new()),
                element("exceptions", Vec::new(), Vec::new()),
            ],
        );
        let result = evaluate_decision_closure(&packet(vec![mapping]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::OpenMissingElements);
        let elements = &result.mappings[0].elements;
        assert_eq!(elements[0].status, ElementClosureStatusV1::Satisfied);
        assert_eq!(elements[1].status, ElementClosureStatusV1::MissingEvidence);
    }

    #[test]
    fn complete_material_element_coverage_closes_one_mapping() {
        let elements = vec![
            element("rule", Vec::new(), vec![support("rule", SupportDispositionV1::DirectSupport)]),
            element("implementation", vec!["rule".to_owned()], vec![support("implementation", SupportDispositionV1::DirectSupport)]),
            element("timing", Vec::new(), vec![support("timing", SupportDispositionV1::DirectSupport)]),
            element("authority", Vec::new(), vec![support("authority", SupportDispositionV1::DirectSupport)]),
            element("exceptions", Vec::new(), vec![support("exceptions", SupportDispositionV1::DirectSupport)]),
        ];
        let result = evaluate_decision_closure(&packet(vec![mapping("complete", elements)]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::Closed);
    }

    #[test]
    fn one_missing_exception_or_documentation_element_keeps_decision_open() {
        let elements = vec![
            element("policy", Vec::new(), vec![support("policy", SupportDispositionV1::DirectSupport)]),
            element("implementation", Vec::new(), vec![support("implementation", SupportDispositionV1::DirectSupport)]),
            element("exceptions", Vec::new(), Vec::new()),
        ];
        let result = evaluate_decision_closure(&packet(vec![mapping("missing-exception", elements)]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::OpenMissingElements);
    }

    #[test]
    fn confirmed_contradiction_blocks_decision_closure() {
        let elements = vec![
            element("policy", Vec::new(), vec![support("policy", SupportDispositionV1::DirectSupport)]),
            element("implementation", Vec::new(), vec![support("implementation", SupportDispositionV1::Contradiction)]),
        ];
        let result = evaluate_decision_closure(&packet(vec![mapping("contradicted", elements)]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::BlockedByContradiction);
    }

    #[test]
    fn partial_or_human_review_support_does_not_close_element() {
        for disposition in [SupportDispositionV1::PartialSupport, SupportDispositionV1::HumanReviewRequired, SupportDispositionV1::InsufficientEvidence] {
            let elements = vec![element("required", Vec::new(), vec![support("partial", disposition)])];
            let result = evaluate_decision_closure(&packet(vec![mapping("partial", elements)]), NOW).unwrap();
            assert_eq!(result.disposition, DecisionClosureDispositionV1::OpenMissingElements);
            assert_eq!(result.mappings[0].elements[0].status, ElementClosureStatusV1::HumanReviewRequired);
        }
    }

    #[test]
    fn wrong_domain_jurisdiction_or_corpus_is_rejected() {
        let mut item = support("wrong-scope", SupportDispositionV1::DirectSupport);
        item.jurisdiction_id = "ZA-WC".to_owned();
        let result = validate_packet(&packet(vec![mapping("wrong-scope", vec![element("required", Vec::new(), vec![item])])]));
        assert_eq!(result, Err(DecisionClosureErrorV1::EvidenceScopeMismatch));
    }

    #[test]
    fn stale_support_result_does_not_count() {
        let mut item = support("stale", SupportDispositionV1::DirectSupport);
        item.valid_until = Some(50);
        let result = evaluate_decision_closure(&packet(vec![mapping("stale", vec![element("required", Vec::new(), vec![item])])]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::OpenMissingElements);
    }

    #[test]
    fn not_applicable_requires_a_digest_bound_authority_decision() {
        let mut item = element("not-applicable", Vec::new(), Vec::new());
        item.applicability = ElementApplicabilityV1::NotApplicable {
            decision_id: "scope-decision".to_owned(),
            decision_digest: [10; 32],
            authority: authority("scope-owner"),
        };
        let result = evaluate_decision_closure(&packet(vec![mapping("n-a", vec![item])]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::Closed);
    }

    #[test]
    fn unresolved_applicability_never_counts_as_covered() {
        let mut item = element("unknown-applicability", Vec::new(), Vec::new());
        item.applicability = ElementApplicabilityV1::Unresolved;
        let result = evaluate_decision_closure(&packet(vec![mapping("unresolved", vec![item])]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::MappingReviewRequired);
    }

    #[test]
    fn unsatisfied_prerequisite_blocks_dependent_element() {
        let elements = vec![
            element("authorization", Vec::new(), Vec::new()),
            element("implementation", vec!["authorization".to_owned()], vec![support("implementation", SupportDispositionV1::DirectSupport)]),
        ];
        let result = evaluate_decision_closure(&packet(vec![mapping("dependency", elements)]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::BlockedByPrerequisite);
        assert_eq!(result.mappings[0].elements[1].status, ElementClosureStatusV1::PrerequisiteUnmet);
    }

    #[test]
    fn dependency_cycle_is_rejected() {
        let elements = vec![
            element("a", vec!["b".to_owned()], Vec::new()),
            element("b", vec!["a".to_owned()], Vec::new()),
        ];
        assert_eq!(validate_packet(&packet(vec![mapping("cycle", elements)])), Err(DecisionClosureErrorV1::DependencyCycle));
    }

    #[test]
    fn accepted_mappings_must_all_be_closed() {
        let complete = mapping("complete", vec![element("element", Vec::new(), vec![support("good", SupportDispositionV1::DirectSupport)])]);
        let open = mapping("alternative", vec![
            element("element", Vec::new(), vec![support("good-2", SupportDispositionV1::DirectSupport)]),
            element("exception", Vec::new(), Vec::new()),
        ]);
        let result = evaluate_decision_closure(&packet(vec![complete, open]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::MappingSensitive);
    }

    #[test]
    fn missing_mapping_approval_prevents_closure() {
        let mut mapping = mapping("unapproved", vec![element("element", Vec::new(), vec![support("good", SupportDispositionV1::DirectSupport)])]);
        mapping.reviews.clear();
        let result = evaluate_decision_closure(&packet(vec![mapping]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::MappingReviewRequired);
    }

    #[test]
    fn mapping_request_changes_prevents_closure() {
        let mut mapping = mapping("requested-changes", vec![element("element", Vec::new(), vec![support("good", SupportDispositionV1::DirectSupport)])]);
        mapping.reviews[0].decision = MappingReviewDecisionV1::RequestChanges;
        let result = evaluate_decision_closure(&packet(vec![mapping]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::MappingReviewRequired);
    }

    #[test]
    fn changed_mapping_digest_invalidates_previous_approval() {
        let mut mapping = mapping("changed", vec![element("element", Vec::new(), vec![support("good", SupportDispositionV1::DirectSupport)])]);
        mapping.mapping_digest = [12; 32];
        let result = validate_packet(&packet(vec![mapping]));
        assert_eq!(result, Err(DecisionClosureErrorV1::ReviewReferenceMismatch));
    }

    #[test]
    fn mapping_quorum_requires_distinct_reviewers_per_role() {
        let mut decision = decision();
        decision.minimum_reviewers_per_role = 2;
        let mut mapping = mapping(
            "two-reviewers",
            vec![element("required", Vec::new(), vec![support("good", SupportDispositionV1::DirectSupport)])],
        );
        mapping.reviews.truncate(1);
        mapping.reviews[0].reviewer_id = "reviewer-one".to_owned();
        let mut packet = packet(vec![mapping]);
        packet.decision = decision.clone();
        assert_eq!(
            evaluate_decision_closure(&packet, NOW).unwrap().disposition,
            DecisionClosureDispositionV1::MappingReviewRequired
        );

        let second = MappingReviewV1 {
            review_id: "review-two".to_owned(),
            mapping_id: "two-reviewers".to_owned(),
            mapping_digest: [9; 32],
            reviewer_id: "reviewer-two".to_owned(),
            role: MappingReviewerRoleV1::DomainExpert,
            authority: authority("second-domain-expert"),
            decision: MappingReviewDecisionV1::Approve,
            reviewed_at: 21,
            valid_until: Some(180),
            withdrawn_at: None,
        };
        packet.mappings[0].reviews.push(second);
        assert_eq!(
            evaluate_decision_closure(&packet, NOW).unwrap().disposition,
            DecisionClosureDispositionV1::Closed
        );
    }

    #[test]
    fn not_applicable_element_does_not_require_irrelevant_prerequisites() {
        let mut excluded = element("conditional-exception", vec!["primary-rule".to_owned()], Vec::new());
        excluded.applicability = ElementApplicabilityV1::NotApplicable {
            decision_id: "exception-does-not-apply".to_owned(),
            decision_digest: [19; 32],
            authority: authority("exception-authority"),
        };
        let result = evaluate_decision_closure(
            &packet(vec![mapping("not-applicable", vec![
                element("primary-rule", Vec::new(), vec![support("rule", SupportDispositionV1::DirectSupport)]),
                excluded,
            ])]),
            NOW,
        ).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::Closed);
        assert_eq!(
            result.mappings[0].elements[1].status,
            ElementClosureStatusV1::NotApplicable
        );
    }

    #[test]
    fn missing_required_mapping_role_prevents_closure() {
        let mut decision = decision();
        decision.required_mapping_roles = vec![
            MappingReviewerRoleV1::DomainExpert,
            MappingReviewerRoleV1::SafetyAuthority,
        ];
        let mut packet = packet(vec![mapping("needs-safety", vec![element("element", Vec::new(), vec![support("good", SupportDispositionV1::DirectSupport)])])]);
        packet.decision = decision;
        let result = evaluate_decision_closure(&packet, NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::MappingReviewRequired);
    }

    #[test]
    fn no_mapping_reports_insufficient_mapping() {
        let result = evaluate_decision_closure(&packet(Vec::new()), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::InsufficientMapping);
    }

    #[test]
    fn changed_decision_digest_invalidates_mapping() {
        let mut mapping = mapping("wrong-decision", vec![element("element", Vec::new(), vec![support("good", SupportDispositionV1::DirectSupport)])]);
        mapping.conclusion_digest = [13; 32];
        assert_eq!(validate_packet(&packet(vec![mapping])), Err(DecisionClosureErrorV1::DecisionReferenceMismatch));
    }

    #[test]
    fn multiple_direct_support_results_can_satisfy_minimum() {
        let mut item = element("two-evidence-sources", Vec::new(), vec![
            support("source-a", SupportDispositionV1::DirectSupport),
            support("source-b", SupportDispositionV1::DirectSupport),
        ]);
        item.minimum_support_results = 2;
        let result = evaluate_decision_closure(&packet(vec![mapping("two-sources", vec![item])]), NOW).unwrap();
        assert_eq!(result.disposition, DecisionClosureDispositionV1::Closed);
    }

    #[test]
    fn duplicate_element_ids_are_rejected() {
        let elements = vec![
            element("duplicate", Vec::new(), Vec::new()),
            element("duplicate", Vec::new(), Vec::new()),
        ];
        assert_eq!(validate_packet(&packet(vec![mapping("duplicate", elements)])), Err(DecisionClosureErrorV1::DuplicateIdentifier));
    }
}
