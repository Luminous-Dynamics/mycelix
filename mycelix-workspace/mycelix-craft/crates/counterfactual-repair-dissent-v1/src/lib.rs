#![forbid(unsafe_code)]

//! Read-only, owner-scoped counterfactual repair guidance and attributed dissent.
//!
//! This crate validates references and completeness claims supplied by domain
//! owners. It does not evaluate evidence truth, freshness, legal applicability,
//! authority, consent, or service outcomes. Hosts must authenticate and replay
//! owner outputs before calling this crate.
//!
//! A repair certificate is guidance only. The types intentionally expose no
//! method to authorize, dispatch, mutate, close, pay, or execute anything.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const COUNTERFACTUAL_REPAIR_DISSENT_SCHEMA_VERSION: u16 = 1;
pub type DigestV1 = [u8; 32];

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ArtifactVersionRefV1 {
    pub artifact_id: String,
    pub revision: u64,
    pub digest: DigestV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecisionRefV1 {
    pub decision_id: String,
    pub revision: u64,
    pub decision_digest: DigestV1,
    pub context_digest: DigestV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum OwnerKindV1 {
    NeedIntake,
    ClaimSupport,
    DecisionClosure,
    EvidenceLineage,
    ImpactQualification,
    DomainOwner,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MaterialCoverageV1 {
    Complete,
    Partial,
    Unknown,
}

/// A host-authenticated reference to one domain owner's completed evaluation.
///
/// coverage_digest commits to the owner's material-element coverage receipt.
/// This crate checks structure and exact binding only; the host must verify
/// signatures, authority, evaluator identity, coverage semantics, and digest.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OwnerEvaluationRefV1 {
    pub evaluation_id: String,
    pub owner: OwnerKindV1,
    pub schema_version: u16,
    pub evaluation_digest: DigestV1,
    pub decision: DecisionRefV1,
    pub context_digest: DigestV1,
    pub coverage: MaterialCoverageV1,
    pub coverage_digest: DigestV1,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct EvidenceRequirementV1 {
    pub evidence_kind: String,
    pub responsible_owner_id: String,
    pub context_digest: DigestV1,
    pub max_age_seconds: Option<u64>,
    pub minimum_independent_origins: u16,
    pub qualification_policy: Option<ArtifactVersionRefV1>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum BlockerKindV1 {
    MissingEvidence,
    StaleEvidence,
    ContradictedEvidence,
    UnresolvedApplicability,
    UnmetPrerequisite,
    MissingAuthority,
    QualifiedReviewRequired,
    ConflictReviewRequired,
    ScopeMismatch,
    Other,
}

/// A material blocker reported by an owner evaluator.
///
/// description_digest binds any human-readable explanation to an immutable
/// explanation artifact. Evidence and authority requirements describe
/// requirements only; they are not grants, credentials, or permission to act.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MaterialBlockerV1 {
    pub blocker_id: String,
    pub material_element: ArtifactVersionRefV1,
    pub source_evaluation_id: String,
    pub source_owner: OwnerKindV1,
    pub kind: BlockerKindV1,
    pub description_digest: DigestV1,
    pub required_evidence: Vec<EvidenceRequirementV1>,
    pub required_authority: Option<ArtifactVersionRefV1>,
    pub prerequisite_blocker_ids: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum GuidanceStepKindV1 {
    CollectEvidence,
    ObtainAuthorityDecision,
    RequestQualifiedReview,
    ResolveConflict,
    RefreshOwnerEvaluation,
    CorrectSourceRecord,
    ReconcileContext,
}

/// A declarative next-step requirement issued by an owner, never an executable
/// command. reassess_by names the owner that must evaluate any new material.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct GuidanceStepV1 {
    pub step_id: String,
    pub proposed_by_evaluation_id: String,
    pub kind: GuidanceStepKindV1,
    pub responsible_owner_id: String,
    pub reassess_by: OwnerKindV1,
    pub context_digest: DigestV1,
    pub satisfies_blocker_ids: Vec<String>,
    pub prerequisite_step_ids: Vec<String>,
    pub required_evidence: Option<EvidenceRequirementV1>,
    pub required_authority: Option<ArtifactVersionRefV1>,
    pub description_digest: DigestV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RepairPacketV1 {
    pub schema_version: u16,
    pub decision: DecisionRefV1,
    /// Exact disposition emitted by the authoritative decision evaluator.
    /// This crate preserves the value and never changes it.
    pub disposition_code: String,
    pub disposition_digest: DigestV1,
    pub owner_evaluations: Vec<OwnerEvaluationRefV1>,
    pub blockers: Vec<MaterialBlockerV1>,
    pub guidance_steps: Vec<GuidanceStepV1>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RepairCertificateV1 {
    pub schema_version: u16,
    pub decision: DecisionRefV1,
    pub disposition_code: String,
    pub disposition_digest: DigestV1,
    pub owner_evaluations: Vec<OwnerEvaluationRefV1>,
    pub blockers: Vec<MaterialBlockerV1>,
    pub guidance_steps: Vec<GuidanceStepV1>,
    /// Explicitly false for this advisory-only reference core.
    pub may_authorize_or_execute: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GuidanceErrorV1 {
    InvalidSchemaVersion,
    EmptyIdentifier,
    InvalidRevision,
    MissingDigest,
    DecisionBindingMismatch,
    ContextBindingMismatch,
    IncompleteOwnerCoverage,
    DuplicateOwnerEvaluation,
    UnknownOwnerEvaluation,
    OwnerMismatch,
    DuplicateBlocker,
    DuplicateGuidanceStep,
    DuplicateRequirement,
    MissingGuidanceForBlocker,
    UnknownBlocker,
    UnknownBlockerPrerequisite,
    BlockerDependencyCycle,
    UnknownStepPrerequisite,
    GuidanceStepCycle,
    InvalidGuidanceRequirement,
    InvalidEvidenceRequirement,
    GuidanceRequirementDoesNotMatchBlocker,
    IncompleteBlockerRequirements,
    InvalidArtifactReference,
    DuplicatePosition,
    DuplicateSummaryPosition,
    UnknownSummaryPosition,
    MissingMaterialPosition,
    PositionBindingMismatch,
    InvalidPositionState,
    InvalidResolution,
    DuplicateResolutionPosition,
    UnknownResolutionPosition,
    AlreadyResolvedPosition,
    ResolutionScopeMismatch,
    SummaryErasesDissent,
    SummaryResolutionMismatch,
}

impl fmt::Display for GuidanceErrorV1 {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{self:?}")
    }
}
impl std::error::Error for GuidanceErrorV1 {}

fn non_empty(value: &str) -> bool {
    !value.is_empty() && value.trim() == value
}
fn valid_digest(digest: &DigestV1) -> bool {
    *digest != [0; 32]
}
fn valid_artifact(reference: &ArtifactVersionRefV1) -> bool {
    non_empty(&reference.artifact_id)
        && reference.revision > 0
        && valid_digest(&reference.digest)
}
fn valid_decision(decision: &DecisionRefV1) -> bool {
    non_empty(&decision.decision_id)
        && decision.revision > 0
        && valid_digest(&decision.decision_digest)
        && valid_digest(&decision.context_digest)
}

fn validate_evidence_requirement(
    requirement: &EvidenceRequirementV1,
    expected_context: &DigestV1,
) -> Result<(), GuidanceErrorV1> {
    if !non_empty(&requirement.evidence_kind)
        || !non_empty(&requirement.responsible_owner_id)
        || requirement.context_digest != *expected_context
        || requirement.minimum_independent_origins == 0
        || requirement.max_age_seconds == Some(0)
    {
        return Err(GuidanceErrorV1::InvalidEvidenceRequirement);
    }
    if requirement
        .qualification_policy
        .as_ref()
        .is_some_and(|reference| !valid_artifact(reference))
    {
        return Err(GuidanceErrorV1::InvalidArtifactReference);
    }
    Ok(())
}

fn validate_unique_strings(values: &[String]) -> Result<(), GuidanceErrorV1> {
    let mut seen = BTreeSet::new();
    for value in values {
        if !non_empty(value) {
            return Err(GuidanceErrorV1::EmptyIdentifier);
        }
        if !seen.insert(value.as_str()) {
            return Err(GuidanceErrorV1::DuplicateRequirement);
        }
    }
    Ok(())
}

fn reject_cycles(
    graph: &BTreeMap<String, Vec<String>>,
    unknown_error: GuidanceErrorV1,
    cycle_error: GuidanceErrorV1,
) -> Result<(), GuidanceErrorV1> {
    fn visit(
        node: &str,
        graph: &BTreeMap<String, Vec<String>>,
        active: &mut BTreeSet<String>,
        done: &mut BTreeSet<String>,
        unknown_error: GuidanceErrorV1,
        cycle_error: GuidanceErrorV1,
    ) -> Result<(), GuidanceErrorV1> {
        if done.contains(node) {
            return Ok(());
        }
        if !active.insert(node.to_owned()) {
            return Err(cycle_error);
        }
        let edges = graph.get(node).ok_or(unknown_error)?;
        for next in edges {
            if !graph.contains_key(next) {
                return Err(unknown_error);
            }
            visit(next, graph, active, done, unknown_error, cycle_error)?;
        }
        active.remove(node);
        done.insert(node.to_owned());
        Ok(())
    }

    let mut active = BTreeSet::new();
    let mut done = BTreeSet::new();
    for node in graph.keys() {
        visit(
            node,
            graph,
            &mut active,
            &mut done,
            unknown_error,
            cycle_error,
        )?;
    }
    Ok(())
}

/// Validate an owner-proposed, read-only repair packet and return a certificate.
///
/// Every owner evaluation must bind to the exact decision and context and attest
/// complete material-element coverage. Every blocker must have at least one
/// owner-proposed guidance step. The function does not invent, authorize, or
/// execute a step, and does not re-run any owner policy.
pub fn build_repair_certificate(
    packet: &RepairPacketV1,
) -> Result<RepairCertificateV1, GuidanceErrorV1> {
    if packet.schema_version != COUNTERFACTUAL_REPAIR_DISSENT_SCHEMA_VERSION {
        return Err(GuidanceErrorV1::InvalidSchemaVersion);
    }
    if !valid_decision(&packet.decision) || !non_empty(&packet.disposition_code) {
        return Err(GuidanceErrorV1::DecisionBindingMismatch);
    }
    if !valid_digest(&packet.disposition_digest) {
        return Err(GuidanceErrorV1::MissingDigest);
    }

    let mut owner_evaluations = BTreeMap::new();
    for evaluation in &packet.owner_evaluations {
        if !non_empty(&evaluation.evaluation_id) {
            return Err(GuidanceErrorV1::EmptyIdentifier);
        }
        if evaluation.schema_version == 0
            || !valid_digest(&evaluation.evaluation_digest)
            || !valid_digest(&evaluation.coverage_digest)
        {
            return Err(GuidanceErrorV1::MissingDigest);
        }
        if evaluation.decision != packet.decision {
            return Err(GuidanceErrorV1::DecisionBindingMismatch);
        }
        if evaluation.context_digest != packet.decision.context_digest {
            return Err(GuidanceErrorV1::ContextBindingMismatch);
        }
        if evaluation.coverage != MaterialCoverageV1::Complete {
            return Err(GuidanceErrorV1::IncompleteOwnerCoverage);
        }
        if owner_evaluations
            .insert(evaluation.evaluation_id.as_str(), evaluation)
            .is_some()
        {
            return Err(GuidanceErrorV1::DuplicateOwnerEvaluation);
        }
    }

    let mut blocker_index = BTreeMap::new();
    let mut blocker_graph = BTreeMap::new();
    for blocker in &packet.blockers {
        if !non_empty(&blocker.blocker_id) {
            return Err(GuidanceErrorV1::EmptyIdentifier);
        }
        if !valid_artifact(&blocker.material_element) {
            return Err(GuidanceErrorV1::InvalidArtifactReference);
        }
        if !valid_digest(&blocker.description_digest) {
            return Err(GuidanceErrorV1::MissingDigest);
        }
        let source = owner_evaluations
            .get(blocker.source_evaluation_id.as_str())
            .ok_or(GuidanceErrorV1::UnknownOwnerEvaluation)?;
        if source.owner != blocker.source_owner {
            return Err(GuidanceErrorV1::OwnerMismatch);
        }
        if source.decision != packet.decision {
            return Err(GuidanceErrorV1::DecisionBindingMismatch);
        }
        if blocker
            .required_authority
            .as_ref()
            .is_some_and(|reference| !valid_artifact(reference))
        {
            return Err(GuidanceErrorV1::InvalidArtifactReference);
        }
        for requirement in &blocker.required_evidence {
            validate_evidence_requirement(requirement, &packet.decision.context_digest)?;
        }
        let requirement_keys = blocker
            .required_evidence
            .iter()
            .map(|requirement| {
                (
                    requirement.evidence_kind.as_str(),
                    requirement.responsible_owner_id.as_str(),
                    requirement.context_digest,
                )
            })
            .collect::<BTreeSet<_>>();
        if requirement_keys.len() != blocker.required_evidence.len() {
            return Err(GuidanceErrorV1::DuplicateRequirement);
        }
        validate_unique_strings(&blocker.prerequisite_blocker_ids)?;
        if blocker_index
            .insert(blocker.blocker_id.as_str(), blocker)
            .is_some()
        {
            return Err(GuidanceErrorV1::DuplicateBlocker);
        }
        blocker_graph.insert(
            blocker.blocker_id.clone(),
            blocker.prerequisite_blocker_ids.clone(),
        );
    }

    for blocker in &packet.blockers {
        for prerequisite in &blocker.prerequisite_blocker_ids {
            if !blocker_index.contains_key(prerequisite.as_str()) {
                return Err(GuidanceErrorV1::UnknownBlockerPrerequisite);
            }
        }
    }
    reject_cycles(
        &blocker_graph,
        GuidanceErrorV1::UnknownBlockerPrerequisite,
        GuidanceErrorV1::BlockerDependencyCycle,
    )?;

    let mut step_index = BTreeMap::new();
    let mut step_graph = BTreeMap::new();
    let mut covered_blockers = BTreeSet::new();
    let mut covered_evidence = BTreeSet::new();
    let mut covered_authority = BTreeSet::new();
    let mut semantic_requirements = BTreeSet::new();
    for step in &packet.guidance_steps {
        if !non_empty(&step.step_id) || !non_empty(&step.responsible_owner_id) {
            return Err(GuidanceErrorV1::EmptyIdentifier);
        }
        if !valid_digest(&step.description_digest) {
            return Err(GuidanceErrorV1::MissingDigest);
        }
        if step.context_digest != packet.decision.context_digest {
            return Err(GuidanceErrorV1::ContextBindingMismatch);
        }
        if !owner_evaluations.contains_key(step.proposed_by_evaluation_id.as_str()) {
            return Err(GuidanceErrorV1::UnknownOwnerEvaluation);
        }
        validate_unique_strings(&step.satisfies_blocker_ids)?;
        validate_unique_strings(&step.prerequisite_step_ids)?;
        if step.satisfies_blocker_ids.is_empty() {
            return Err(GuidanceErrorV1::MissingGuidanceForBlocker);
        }
        for blocker_id in &step.satisfies_blocker_ids {
            let blocker = blocker_index
                .get(blocker_id.as_str())
                .ok_or(GuidanceErrorV1::UnknownBlocker)?;
            covered_blockers.insert(blocker_id.as_str());
            if let Some(requirement) = &step.required_evidence {
                if !blocker.required_evidence.contains(requirement) {
                    return Err(GuidanceErrorV1::GuidanceRequirementDoesNotMatchBlocker);
                }
                covered_evidence.insert((blocker_id.clone(), requirement.clone()));
            }
            if let Some(authority) = &step.required_authority {
                if blocker.required_authority.as_ref() != Some(authority) {
                    return Err(GuidanceErrorV1::GuidanceRequirementDoesNotMatchBlocker);
                }
                covered_authority.insert((blocker_id.clone(), authority.clone()));
            }
        }
        if step
            .required_authority
            .as_ref()
            .is_some_and(|reference| !valid_artifact(reference))
        {
            return Err(GuidanceErrorV1::InvalidArtifactReference);
        }
        if let Some(requirement) = &step.required_evidence {
            validate_evidence_requirement(requirement, &packet.decision.context_digest)?;
        }
        match step.kind {
            GuidanceStepKindV1::CollectEvidence | GuidanceStepKindV1::RefreshOwnerEvaluation => {
                if step.required_evidence.is_none() {
                    return Err(GuidanceErrorV1::InvalidGuidanceRequirement);
                }
            }
            GuidanceStepKindV1::ObtainAuthorityDecision => {
                if step.required_authority.is_none() {
                    return Err(GuidanceErrorV1::InvalidGuidanceRequirement);
                }
            }
            GuidanceStepKindV1::RequestQualifiedReview => {
                if step.required_authority.is_none() {
                    return Err(GuidanceErrorV1::InvalidGuidanceRequirement);
                }
            }
            GuidanceStepKindV1::ResolveConflict
            | GuidanceStepKindV1::CorrectSourceRecord
            | GuidanceStepKindV1::ReconcileContext => {}
        }

        let signature = (
            step.kind,
            step.responsible_owner_id.as_str(),
            step.reassess_by,
            step.context_digest,
            step.required_evidence.as_ref(),
            step.required_authority.as_ref(),
            step.prerequisite_step_ids.as_slice(),
            step.description_digest,
        );
        if !semantic_requirements.insert(signature) {
            return Err(GuidanceErrorV1::DuplicateGuidanceStep);
        }
        if step_index.insert(step.step_id.as_str(), step).is_some() {
            return Err(GuidanceErrorV1::DuplicateGuidanceStep);
        }
        step_graph.insert(
            step.step_id.clone(),
            step.prerequisite_step_ids.clone(),
        );
    }

    for step in &packet.guidance_steps {
        for prerequisite in &step.prerequisite_step_ids {
            if !step_index.contains_key(prerequisite.as_str()) {
                return Err(GuidanceErrorV1::UnknownStepPrerequisite);
            }
        }
    }
    reject_cycles(
        &step_graph,
        GuidanceErrorV1::UnknownStepPrerequisite,
        GuidanceErrorV1::GuidanceStepCycle,
    )?;

    for blocker in &packet.blockers {
        if !covered_blockers.contains(blocker.blocker_id.as_str()) {
            return Err(GuidanceErrorV1::MissingGuidanceForBlocker);
        }
        if blocker.required_evidence.iter().any(|requirement| {
            !covered_evidence.contains(&(blocker.blocker_id.clone(), requirement.clone()))
        }) || blocker.required_authority.as_ref().is_some_and(|authority| {
            !covered_authority.contains(&(blocker.blocker_id.clone(), authority.clone()))
        }) {
            return Err(GuidanceErrorV1::IncompleteBlockerRequirements);
        }
    }

    let mut certificate = RepairCertificateV1 {
        schema_version: packet.schema_version,
        decision: packet.decision.clone(),
        disposition_code: packet.disposition_code.clone(),
        disposition_digest: packet.disposition_digest,
        owner_evaluations: packet.owner_evaluations.clone(),
        blockers: packet.blockers.clone(),
        guidance_steps: packet.guidance_steps.clone(),
        may_authorize_or_execute: false,
    };
    certificate.owner_evaluations.sort_by(|left, right| {
        left.evaluation_id.cmp(&right.evaluation_id)
    });
    certificate.blockers.sort_by(|left, right| {
        left.blocker_id.cmp(&right.blocker_id)
    });
    certificate.guidance_steps.sort_by(|left, right| {
        left.step_id.cmp(&right.step_id)
    });
    Ok(certificate)
}

// ---- Attributed position and dissent-preservation contract ----

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PositionStateV1 {
    Active,
    Disputed,
    Resolved,
    Withdrawn,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StakeholderPositionV1 {
    pub position_id: String,
    pub revision: u64,
    pub proposal: ArtifactVersionRefV1,
    pub stakeholder_group_id: String,
    /// The host verifies issuer, permissions, visibility and speaking authority.
    pub attribution: ArtifactVersionRefV1,
    pub statement_digest: DigestV1,
    pub provenance: Vec<ArtifactVersionRefV1>,
    /// Exact outcome, requirement, or material-element identifiers affected.
    pub material_scope_ids: Vec<String>,
    pub state: PositionStateV1,
    /// Exact authenticated resolution record when the position is resolved or withdrawn.
    pub resolution: Option<PositionResolutionV1>,
    pub position_digest: DigestV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PositionResolutionV1 {
    pub resolution_id: String,
    pub resolution_artifact: ArtifactVersionRefV1,
    pub proposal: ArtifactVersionRefV1,
    pub position_ids: Vec<String>,
    /// Decision and authority artifacts are exact references; host authenticates.
    pub decision: ArtifactVersionRefV1,
    pub authority: ArtifactVersionRefV1,
    pub resolution_digest: DigestV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SummaryPositionDispositionV1 {
    Preserved,
    ExplicitlyResolved,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SummaryPositionRefV1 {
    pub position_id: String,
    pub position_digest: DigestV1,
    pub disposition: SummaryPositionDispositionV1,
    pub resolution_id: Option<String>,
    pub resolution_ref: Option<ArtifactVersionRefV1>,
    pub resolution_decision: Option<ArtifactVersionRefV1>,
}

fn validate_position(position: &StakeholderPositionV1) -> Result<(), GuidanceErrorV1> {
    if !non_empty(&position.position_id) || !non_empty(&position.stakeholder_group_id) {
        return Err(GuidanceErrorV1::EmptyIdentifier);
    }
    if position.revision == 0 {
        return Err(GuidanceErrorV1::InvalidRevision);
    }
    if !valid_artifact(&position.proposal)
        || !valid_artifact(&position.attribution)
        || !valid_digest(&position.statement_digest)
        || !valid_digest(&position.position_digest)
    {
        return Err(GuidanceErrorV1::InvalidArtifactReference);
    }
    if position.provenance.is_empty() || position.material_scope_ids.is_empty() {
        return Err(GuidanceErrorV1::PositionBindingMismatch);
    }
    for reference in &position.provenance {
        if !valid_artifact(reference) {
            return Err(GuidanceErrorV1::InvalidArtifactReference);
        }
    }
    validate_unique_strings(&position.material_scope_ids)?;
    match position.state {
        PositionStateV1::Active | PositionStateV1::Disputed => {
            if position.resolution.is_some() {
                return Err(GuidanceErrorV1::InvalidPositionState);
            }
        }
        PositionStateV1::Resolved | PositionStateV1::Withdrawn => {
            let resolution = position
                .resolution
                .as_ref()
                .ok_or(GuidanceErrorV1::InvalidPositionState)?;
            if !non_empty(&resolution.resolution_id)
                || !valid_artifact(&resolution.resolution_artifact)
                || resolution.resolution_artifact.artifact_id != resolution.resolution_id
                || resolution.resolution_artifact.digest != resolution.resolution_digest
                || resolution.proposal != position.proposal
                || !resolution.position_ids.contains(&position.position_id)
                || !valid_artifact(&resolution.decision)
                || !valid_artifact(&resolution.authority)
                || !valid_digest(&resolution.resolution_digest)
            {
                return Err(GuidanceErrorV1::PositionBindingMismatch);
            }
            validate_unique_strings(&resolution.position_ids)?;
        }
    }
    Ok(())
}

/// Validate an explicit community resolution against exact attributed positions.
///
/// Authentication and the social/legal sufficiency of the decision remain host
/// responsibilities. This function enforces revision/scope/reference exactness.
pub fn validate_position_resolution(
    resolution: &PositionResolutionV1,
    positions: &[StakeholderPositionV1],
) -> Result<(), GuidanceErrorV1> {
    if !non_empty(&resolution.resolution_id)
        || !valid_artifact(&resolution.resolution_artifact)
        || resolution.resolution_artifact.artifact_id != resolution.resolution_id
        || resolution.resolution_artifact.digest != resolution.resolution_digest
        || !valid_artifact(&resolution.proposal)
        || !valid_artifact(&resolution.decision)
        || !valid_artifact(&resolution.authority)
        || !valid_digest(&resolution.resolution_digest)
        || resolution.position_ids.is_empty()
    {
        return Err(GuidanceErrorV1::InvalidResolution);
    }
    validate_unique_strings(&resolution.position_ids)?;
    let indexed = positions
        .iter()
        .map(|position| (position.position_id.as_str(), position))
        .collect::<BTreeMap<_, _>>();
    if indexed.len() != positions.len() {
        return Err(GuidanceErrorV1::DuplicatePosition);
    }
    for position_id in &resolution.position_ids {
        let position = indexed
            .get(position_id.as_str())
            .ok_or(GuidanceErrorV1::UnknownResolutionPosition)?;
        validate_position(position)?;
        if position.proposal != resolution.proposal {
            return Err(GuidanceErrorV1::ResolutionScopeMismatch);
        }
        if matches!(position.state, PositionStateV1::Resolved | PositionStateV1::Withdrawn) {
            return Err(GuidanceErrorV1::AlreadyResolvedPosition);
        }
    }
    Ok(())
}

/// Ensure every material position is represented exactly once in a summary.
///
/// Active and disputed positions must remain explicitly preserved. Resolved or
/// withdrawn positions may be summarized only with an exact resolution ID and
/// decision revision. No consensus is inferred from omission or compression.
pub fn validate_summary_preserves_positions(
    positions: &[StakeholderPositionV1],
    summary: &[SummaryPositionRefV1],
) -> Result<(), GuidanceErrorV1> {
    let mut positions_by_id = BTreeMap::new();
    for position in positions {
        validate_position(position)?;
        if positions_by_id
            .insert(position.position_id.as_str(), position)
            .is_some()
        {
            return Err(GuidanceErrorV1::DuplicatePosition);
        }
    }

    let mut summary_by_id = BTreeMap::new();
    for item in summary {
        if !non_empty(&item.position_id) || !valid_digest(&item.position_digest) {
            return Err(GuidanceErrorV1::PositionBindingMismatch);
        }
        if summary_by_id.insert(item.position_id.as_str(), item).is_some() {
            return Err(GuidanceErrorV1::DuplicateSummaryPosition);
        }
        if !positions_by_id.contains_key(item.position_id.as_str()) {
            return Err(GuidanceErrorV1::UnknownSummaryPosition);
        }
    }

    for position in positions.iter().filter(|position| !position.material_scope_ids.is_empty()) {
        let item = summary_by_id
            .get(position.position_id.as_str())
            .ok_or(GuidanceErrorV1::MissingMaterialPosition)?;
        if item.position_digest != position.position_digest {
            return Err(GuidanceErrorV1::PositionBindingMismatch);
        }
        match position.state {
            PositionStateV1::Active | PositionStateV1::Disputed => {
                if item.disposition != SummaryPositionDispositionV1::Preserved
                    || item.resolution_id.is_some()
                    || item.resolution_ref.is_some()
                    || item.resolution_decision.is_some()
                {
                    return Err(GuidanceErrorV1::SummaryErasesDissent);
                }
            }
            PositionStateV1::Resolved | PositionStateV1::Withdrawn => {
                let resolution = position
                    .resolution
                    .as_ref()
                    .ok_or(GuidanceErrorV1::InvalidPositionState)?;
                if item.disposition != SummaryPositionDispositionV1::ExplicitlyResolved
                    || item.resolution_id.as_deref() != Some(resolution.resolution_id.as_str())
                    || item.resolution_ref.as_ref() != Some(&resolution.resolution_artifact)
                    || item.resolution_decision.as_ref() != Some(&resolution.decision)
                {
                    return Err(GuidanceErrorV1::SummaryResolutionMismatch);
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn artifact(id: &str, revision: u64, byte: u8) -> ArtifactVersionRefV1 {
        ArtifactVersionRefV1 {
            artifact_id: id.to_owned(),
            revision,
            digest: [byte; 32],
        }
    }

    fn decision() -> DecisionRefV1 {
        DecisionRefV1 {
            decision_id: "community-water-decision".to_owned(),
            revision: 2,
            decision_digest: [1; 32],
            context_digest: [2; 32],
        }
    }

    fn owner(id: &str, kind: OwnerKindV1) -> OwnerEvaluationRefV1 {
        OwnerEvaluationRefV1 {
            evaluation_id: id.to_owned(),
            owner: kind,
            schema_version: 1,
            evaluation_digest: [3; 32],
            decision: decision(),
            context_digest: [2; 32],
            coverage: MaterialCoverageV1::Complete,
            coverage_digest: [4; 32],
        }
    }

    fn evidence_requirement(kind: &str) -> EvidenceRequirementV1 {
        EvidenceRequirementV1 {
            evidence_kind: kind.to_owned(),
            responsible_owner_id: "qualified-observation-team".to_owned(),
            context_digest: [2; 32],
            max_age_seconds: Some(3600),
            minimum_independent_origins: 1,
            qualification_policy: Some(artifact("observation-policy", 1, 5)),
        }
    }

    fn blocker(id: &str, requirement: EvidenceRequirementV1) -> MaterialBlockerV1 {
        MaterialBlockerV1 {
            blocker_id: id.to_owned(),
            material_element: artifact(&format!("element-{id}"), 1, 6),
            source_evaluation_id: "closure-eval".to_owned(),
            source_owner: OwnerKindV1::DecisionClosure,
            kind: BlockerKindV1::MissingEvidence,
            description_digest: [7; 32],
            required_evidence: vec![requirement],
            required_authority: None,
            prerequisite_blocker_ids: Vec::new(),
        }
    }

    fn step(id: &str, blocker_id: &str, requirement: EvidenceRequirementV1) -> GuidanceStepV1 {
        GuidanceStepV1 {
            step_id: id.to_owned(),
            proposed_by_evaluation_id: "closure-eval".to_owned(),
            kind: GuidanceStepKindV1::CollectEvidence,
            responsible_owner_id: "qualified-observation-team".to_owned(),
            reassess_by: OwnerKindV1::ClaimSupport,
            context_digest: [2; 32],
            satisfies_blocker_ids: vec![blocker_id.to_owned()],
            prerequisite_step_ids: Vec::new(),
            required_evidence: Some(requirement),
            required_authority: None,
            description_digest: [8; 32],
        }
    }

    fn valid_packet() -> RepairPacketV1 {
        let requirement = evidence_requirement("independent-pressure-observation");
        RepairPacketV1 {
            schema_version: 1,
            decision: decision(),
            disposition_code: "open_missing_elements".to_owned(),
            disposition_digest: [9; 32],
            owner_evaluations: vec![owner(
                "closure-eval",
                OwnerKindV1::DecisionClosure,
            )],
            blockers: vec![blocker("pressure-restoration", requirement.clone())],
            guidance_steps: vec![step("collect-pressure", "pressure-restoration", requirement)],
        }
    }

    #[test]
    fn builds_read_only_certificate_without_changing_owner_disposition() {
        let packet = valid_packet();
        let certificate = build_repair_certificate(&packet).unwrap();
        assert_eq!(certificate.disposition_code, "open_missing_elements");
        assert_eq!(certificate.disposition_digest, packet.disposition_digest);
        assert!(!certificate.may_authorize_or_execute);
        assert_eq!(certificate.blockers, packet.blockers);
        assert_eq!(certificate.guidance_steps, packet.guidance_steps);
    }

    #[test]
    fn rejects_owner_evaluation_with_partial_material_coverage() {
        let mut packet = valid_packet();
        packet.owner_evaluations[0].coverage = MaterialCoverageV1::Partial;
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::IncompleteOwnerCoverage)
        );
    }

    #[test]
    fn rejects_owner_output_from_a_different_decision_revision() {
        let mut packet = valid_packet();
        packet.owner_evaluations[0].decision.revision = 1;
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::DecisionBindingMismatch)
        );
    }

    #[test]
    fn rejects_owner_output_from_a_different_context() {
        let mut packet = valid_packet();
        packet.owner_evaluations[0].context_digest = [11; 32];
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::ContextBindingMismatch)
        );
    }

    #[test]
    fn every_material_blocker_requires_at_least_one_guidance_step() {
        let mut packet = valid_packet();
        packet.guidance_steps.clear();
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::MissingGuidanceForBlocker)
        );
    }

    #[test]
    fn guidance_cannot_reference_an_unknown_blocker() {
        let mut packet = valid_packet();
        packet.guidance_steps[0].satisfies_blocker_ids = vec!["invented-blocker".to_owned()];
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::UnknownBlocker)
        );
    }

    #[test]
    fn guidance_cannot_be_proposed_by_an_unknown_owner_evaluation() {
        let mut packet = valid_packet();
        packet.guidance_steps[0].proposed_by_evaluation_id = "unknown-eval".to_owned();
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::UnknownOwnerEvaluation)
        );
    }

    #[test]
    fn guidance_must_bind_to_the_exact_context() {
        let mut packet = valid_packet();
        packet.guidance_steps[0].context_digest = [12; 32];
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::ContextBindingMismatch)
        );
    }

    #[test]
    fn evidence_collection_step_requires_a_typed_evidence_requirement() {
        let mut packet = valid_packet();
        packet.guidance_steps[0].required_evidence = None;
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::InvalidGuidanceRequirement)
        );
    }

    #[test]
    fn authority_step_requires_an_exact_authority_policy_reference() {
        let mut packet = valid_packet();
        packet.guidance_steps[0].kind = GuidanceStepKindV1::ObtainAuthorityDecision;
        packet.guidance_steps[0].required_evidence = None;
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::InvalidGuidanceRequirement)
        );
    }

    #[test]
    fn rejects_blocker_dependency_cycles() {
        let mut packet = valid_packet();
        packet.blockers[0].prerequisite_blocker_ids = vec!["pressure-restoration".to_owned()];
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::BlockerDependencyCycle)
        );
    }

    #[test]
    fn rejects_guidance_step_dependency_cycles() {
        let mut packet = valid_packet();
        packet.guidance_steps[0].prerequisite_step_ids = vec!["collect-pressure".to_owned()];
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::GuidanceStepCycle)
        );
    }

    #[test]
    fn one_owner_step_may_address_multiple_exact_blockers() {
        let mut packet = valid_packet();
        let second_requirement = evidence_requirement("independent-pressure-observation");
        packet.blockers.push(blocker("pressure-duration", second_requirement));
        packet.guidance_steps[0]
            .satisfies_blocker_ids
            .push("pressure-duration".to_owned());
        assert!(build_repair_certificate(&packet).is_ok());
    }

    #[test]
    fn every_evidence_requirement_must_be_satisfied_by_an_exact_step() {
        let mut packet = valid_packet();
        packet.blockers[0]
            .required_evidence
            .push(evidence_requirement("pressure-duration-observation"));
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::IncompleteBlockerRequirements)
        );
    }

    #[test]
    fn guidance_cannot_substitute_a_different_evidence_kind() {
        let mut packet = valid_packet();
        packet.guidance_steps[0].required_evidence =
            Some(evidence_requirement("provider-completion-assertion"));
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::GuidanceRequirementDoesNotMatchBlocker)
        );
    }

    #[test]
    fn guidance_cannot_expand_or_substitute_the_authority_reference() {
        let mut packet = valid_packet();
        let required = artifact("local-authority-policy", 3, 34);
        packet.blockers[0].required_authority = Some(required);
        packet.guidance_steps[0].kind = GuidanceStepKindV1::ObtainAuthorityDecision;
        packet.guidance_steps[0].required_evidence = None;
        packet.guidance_steps[0].required_authority =
            Some(artifact("different-authority-policy", 1, 35));
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::GuidanceRequirementDoesNotMatchBlocker)
        );
    }

    #[test]
    fn duplicate_semantic_guidance_steps_must_be_consolidated() {
        let mut packet = valid_packet();
        let mut duplicate = packet.guidance_steps[0].clone();
        duplicate.step_id = "duplicate-step".to_owned();
        packet.guidance_steps.push(duplicate);
        assert_eq!(
            build_repair_certificate(&packet),
            Err(GuidanceErrorV1::DuplicateGuidanceStep)
        );
    }

    fn position(
        id: &str,
        state: PositionStateV1,
        material_scope: &str,
    ) -> StakeholderPositionV1 {
        StakeholderPositionV1 {
            position_id: id.to_owned(),
            revision: 1,
            proposal: artifact("water-need-proposal", 3, 20),
            stakeholder_group_id: format!("group-{id}"),
            attribution: artifact(&format!("attribution-{id}"), 1, 21),
            statement_digest: [22; 32],
            provenance: vec![artifact(&format!("source-{id}"), 1, 23)],
            material_scope_ids: vec![material_scope.to_owned()],
            state,
            resolution: None,
            position_digest: [24; 32],
        }
    }

    fn resolved_position(id: &str, material_scope: &str) -> StakeholderPositionV1 {
        let mut position = position(id, PositionStateV1::Resolved, material_scope);
        position.resolution = Some(PositionResolutionV1 {
            resolution_id: "community-resolution-1".to_owned(),
            resolution_artifact: artifact("community-resolution-1", 1, 29),
            proposal: position.proposal.clone(),
            position_ids: vec![position.position_id.clone()],
            decision: artifact("community-decision", 1, 27),
            authority: artifact("community-authority", 1, 28),
            resolution_digest: [29; 32],
        });
        position
    }

    fn summary_item(
        position_id: &str,
        disposition: SummaryPositionDispositionV1,
    ) -> SummaryPositionRefV1 {
        SummaryPositionRefV1 {
            position_id: position_id.to_owned(),
            position_digest: [24; 32],
            disposition,
            resolution_id: None,
            resolution_ref: None,
            resolution_decision: None,
        }
    }

    #[test]
    fn summary_must_preserve_every_material_active_position() {
        let positions = vec![
            position("affordability", PositionStateV1::Active, "cost-outcome"),
            position("reliability", PositionStateV1::Active, "reliability-outcome"),
        ];
        let summary = vec![summary_item(
            "affordability",
            SummaryPositionDispositionV1::Preserved,
        )];
        assert_eq!(
            validate_summary_preserves_positions(&positions, &summary),
            Err(GuidanceErrorV1::MissingMaterialPosition)
        );
    }

    #[test]
    fn summary_cannot_merge_two_positions_into_one_identity() {
        let positions = vec![
            position("affordability", PositionStateV1::Active, "cost-outcome"),
            position("reliability", PositionStateV1::Disputed, "reliability-outcome"),
        ];
        let summary = vec![summary_item(
            "affordability",
            SummaryPositionDispositionV1::Preserved,
        )];
        assert_eq!(
            validate_summary_preserves_positions(&positions, &summary),
            Err(GuidanceErrorV1::MissingMaterialPosition)
        );
    }

    #[test]
    fn summary_must_bind_to_the_exact_position_digest() {
        let positions = vec![position(
            "affordability",
            PositionStateV1::Active,
            "cost-outcome",
        )];
        let mut item = summary_item(
            "affordability",
            SummaryPositionDispositionV1::Preserved,
        );
        item.position_digest = [25; 32];
        assert_eq!(
            validate_summary_preserves_positions(&positions, &[item]),
            Err(GuidanceErrorV1::PositionBindingMismatch)
        );
    }

    #[test]
    fn unresolved_dissent_cannot_be_marked_as_resolved() {
        let positions = vec![position(
            "affordability",
            PositionStateV1::Disputed,
            "cost-outcome",
        )];
        let mut item = summary_item(
            "affordability",
            SummaryPositionDispositionV1::ExplicitlyResolved,
        );
        item.resolution_id = Some("model-consensus".to_owned());
        item.resolution_decision = Some(artifact("model-output", 1, 26));
        assert_eq!(
            validate_summary_preserves_positions(&positions, &[item]),
            Err(GuidanceErrorV1::SummaryErasesDissent)
        );
    }

    #[test]
    fn resolved_position_requires_explicit_resolution_reference_in_summary() {
        let positions = vec![resolved_position("affordability", "cost-outcome")];
        let item = summary_item(
            "affordability",
            SummaryPositionDispositionV1::ExplicitlyResolved,
        );
        assert_eq!(
            validate_summary_preserves_positions(&positions, &[item]),
            Err(GuidanceErrorV1::SummaryResolutionMismatch)
        );
    }

    #[test]
    fn valid_explicit_resolution_is_preserved_as_history() {
        let positions = vec![position(
            "affordability",
            PositionStateV1::Active,
            "cost-outcome",
        )];
        let resolution = PositionResolutionV1 {
            resolution_id: "community-resolution-1".to_owned(),
            resolution_artifact: artifact("community-resolution-1", 1, 29),
            proposal: artifact("water-need-proposal", 3, 20),
            position_ids: vec!["affordability".to_owned()],
            decision: artifact("community-decision", 1, 27),
            authority: artifact("community-authority", 1, 28),
            resolution_digest: [29; 32],
        };
        assert!(validate_position_resolution(&resolution, &positions).is_ok());
    }

    #[test]
    fn resolution_cannot_cross_proposal_revision() {
        let positions = vec![position(
            "affordability",
            PositionStateV1::Active,
            "cost-outcome",
        )];
        let resolution = PositionResolutionV1 {
            resolution_id: "community-resolution-1".to_owned(),
            resolution_artifact: artifact("community-resolution-1", 1, 29),
            proposal: artifact("water-need-proposal", 2, 30),
            position_ids: vec!["affordability".to_owned()],
            decision: artifact("community-decision", 1, 27),
            authority: artifact("community-authority", 1, 28),
            resolution_digest: [29; 32],
        };
        assert_eq!(
            validate_position_resolution(&resolution, &positions),
            Err(GuidanceErrorV1::ResolutionScopeMismatch)
        );
    }

    #[test]
    fn resolution_cannot_target_an_unknown_position() {
        let positions = vec![position(
            "affordability",
            PositionStateV1::Active,
            "cost-outcome",
        )];
        let resolution = PositionResolutionV1 {
            resolution_id: "community-resolution-1".to_owned(),
            resolution_artifact: artifact("community-resolution-1", 1, 29),
            proposal: artifact("water-need-proposal", 3, 20),
            position_ids: vec!["invented-position".to_owned()],
            decision: artifact("community-decision", 1, 27),
            authority: artifact("community-authority", 1, 28),
            resolution_digest: [29; 32],
        };
        assert_eq!(
            validate_position_resolution(&resolution, &positions),
            Err(GuidanceErrorV1::UnknownResolutionPosition)
        );
    }

    #[test]
    fn resolved_position_cannot_be_resolved_again() {
        let positions = vec![resolved_position("affordability", "cost-outcome")];
        let resolution = PositionResolutionV1 {
            resolution_id: "community-resolution-2".to_owned(),
            resolution_artifact: artifact("community-resolution-2", 1, 32),
            proposal: artifact("water-need-proposal", 3, 20),
            position_ids: vec!["affordability".to_owned()],
            decision: artifact("community-decision-2", 1, 31),
            authority: artifact("community-authority", 1, 28),
            resolution_digest: [32; 32],
        };
        assert_eq!(
            validate_position_resolution(&resolution, &positions),
            Err(GuidanceErrorV1::AlreadyResolvedPosition)
        );
    }

    #[test]
    fn duplicate_positions_are_rejected() {
        let position = position("affordability", PositionStateV1::Active, "cost-outcome");
        let resolution = PositionResolutionV1 {
            resolution_id: "community-resolution-1".to_owned(),
            resolution_artifact: artifact("community-resolution-1", 1, 29),
            proposal: artifact("water-need-proposal", 3, 20),
            position_ids: vec!["affordability".to_owned()],
            decision: artifact("community-decision", 1, 27),
            authority: artifact("community-authority", 1, 28),
            resolution_digest: [29; 32],
        };
        assert_eq!(
            validate_position_resolution(&resolution, &[position.clone(), position]),
            Err(GuidanceErrorV1::DuplicatePosition)
        );
    }

    #[test]
    fn resolution_requires_unique_position_ids() {
        let positions = vec![position(
            "affordability",
            PositionStateV1::Active,
            "cost-outcome",
        )];
        let resolution = PositionResolutionV1 {
            resolution_id: "community-resolution-1".to_owned(),
            resolution_artifact: artifact("community-resolution-1", 1, 29),
            proposal: artifact("water-need-proposal", 3, 20),
            position_ids: vec!["affordability".to_owned(), "affordability".to_owned()],
            decision: artifact("community-decision", 1, 27),
            authority: artifact("community-authority", 1, 28),
            resolution_digest: [29; 32],
        };
        assert_eq!(
            validate_position_resolution(&resolution, &positions),
            Err(GuidanceErrorV1::DuplicateRequirement)
        );
    }

    #[test]
    fn valid_resolved_position_must_reference_its_explicit_decision() {
        let position = resolved_position("affordability", "cost-outcome");
        let mut item = summary_item(
            "affordability",
            SummaryPositionDispositionV1::ExplicitlyResolved,
        );
        item.resolution_id = Some("community-resolution-1".to_owned());
        item.resolution_ref = Some(artifact("community-resolution-1", 1, 29));
        item.resolution_decision = Some(artifact("community-decision", 1, 27));
        assert!(validate_summary_preserves_positions(&[position.clone()], &[item.clone()]).is_ok());
        let mut changed = position;
        changed.position_digest = [33; 32];
        assert_eq!(
            validate_summary_preserves_positions(&[changed], &[item]),
            Err(GuidanceErrorV1::PositionBindingMismatch)
        );
    }
}
