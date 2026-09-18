// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![no_std]

extern crate alloc;

use alloc::collections::BTreeSet;
use alloc::vec::Vec;
use mycelix_ai_lineage::OperationalSubjectRef;

pub const PROTOCOL_VERSION: &str = "mycelix-amsap-subject-topology-v0.1";
pub const MAX_COMPONENTS: usize = 1024;
pub const MAX_EDGES: usize = 4096;
pub const MAX_ALTERNATIVES: usize = 128;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IdentifierKind {
    SubjectBoundary,
    Component,
    IndividuationHypothesis,
    TopologyManifestCommitment,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TopologyError {
    ZeroIdentifier(IdentifierKind),
    ZeroEpoch,
    EmptyComponentSet,
    TooManyComponents,
    TooManyEdges,
    TooManyAlternatives,
    DuplicateComponentId,
    DuplicateOperationalComponent,
    NonCanonicalComponentOrder,
    DanglingEdgeEndpoint,
    SelfEdge,
    DuplicateEdge,
    NonCanonicalEdgeOrder,
    DuplicateAlternativeHypothesis,
    DuplicateAlternativeBoundary,
    AlternativeReusesCurrentHypothesis,
    AlternativeReusesCurrentBoundary,
    NonCanonicalAlternativeOrder,
    InvalidSingleRuntimeCardinality,
    SingleRuntimeComponentNotOperational,
    SingleRuntimeAnchorMismatch,
    AnchorMissingFromGraph,
    BoundaryEquivocation,
}

macro_rules! topology_id_type {
    ($name:ident, $kind:expr) => {
        #[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
        pub struct $name([u8; 32]);

        impl $name {
            pub fn new(bytes: [u8; 32]) -> Result<Self, TopologyError> {
                if bytes == [0; 32] {
                    return Err(TopologyError::ZeroIdentifier($kind));
                }
                Ok(Self(bytes))
            }

            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

topology_id_type!(
    SubjectBoundaryId,
    IdentifierKind::SubjectBoundary
);
topology_id_type!(ComponentId, IdentifierKind::Component);
topology_id_type!(
    IndividuationHypothesisId,
    IdentifierKind::IndividuationHypothesis
);
topology_id_type!(
    TopologyManifestCommitment,
    IdentifierKind::TopologyManifestCommitment
);

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct TopologyEpoch(u64);

impl TopologyEpoch {
    pub fn new(value: u64) -> Result<Self, TopologyError> {
        if value == 0 {
            return Err(TopologyError::ZeroEpoch);
        }
        Ok(Self(value))
    }

    pub const fn get(self) -> u64 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CandidateKind {
    SingleRuntimeInstance,
    ScaffoldedAgent,
    CompositeAgent,
    MultiAgentCollective,
    ServicePool,
    ForkFamily,
    DistributedExecution,
    ConditionalExpertSystem,
    TrainingProcess,
    TrainingPopulation,
    UnknownComposite,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MembershipPolicy {
    Fixed,
    Dynamic,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ComponentRef {
    Operational(OperationalSubjectRef),
    Opaque,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ComponentNode {
    pub component_id: ComponentId,
    pub component_ref: ComponentRef,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum CompositionEdgeKind {
    Contains,
    StateSharing,
    Memory,
    Control,
    Communication,
    Routing,
    ResourceSharing,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CompositionEdge {
    pub from: ComponentId,
    pub to: ComponentId,
    pub kind: CompositionEdgeKind,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AlternativeBoundaryRef {
    pub hypothesis_id: IndividuationHypothesisId,
    pub boundary_id: SubjectBoundaryId,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SubjectBoundaryRef {
    pub boundary_id: SubjectBoundaryId,
    pub individuation_hypothesis_id: IndividuationHypothesisId,
    pub epoch: TopologyEpoch,
    pub topology_manifest_commitment: TopologyManifestCommitment,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CandidateSubjectBoundary {
    pub boundary_id: SubjectBoundaryId,
    pub individuation_hypothesis_id: IndividuationHypothesisId,
    pub candidate_kind: CandidateKind,
    pub epoch: TopologyEpoch,
    pub membership_policy: MembershipPolicy,
    pub anchor_subject: Option<OperationalSubjectRef>,
    pub topology_manifest_commitment: TopologyManifestCommitment,
    pub components: Vec<ComponentNode>,
    pub edges: Vec<CompositionEdge>,
    pub preregistered_alternatives: Vec<AlternativeBoundaryRef>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ValidatedCandidateSubjectBoundary {
    boundary: CandidateSubjectBoundary,
}

impl ValidatedCandidateSubjectBoundary {
    pub const fn boundary(&self) -> &CandidateSubjectBoundary {
        &self.boundary
    }

    pub fn subject_ref(&self) -> SubjectBoundaryRef {
        SubjectBoundaryRef {
            boundary_id: self.boundary.boundary_id,
            individuation_hypothesis_id: self.boundary.individuation_hypothesis_id,
            epoch: self.boundary.epoch,
            topology_manifest_commitment: self.boundary.topology_manifest_commitment,
        }
    }

    pub fn component_count(&self) -> usize {
        self.boundary.components.len()
    }

    pub fn operational_component_count(&self) -> usize {
        self.boundary
            .components
            .iter()
            .filter(|node| matches!(node.component_ref, ComponentRef::Operational(_)))
            .count()
    }

    pub const fn establishes_consciousness(&self) -> bool {
        false
    }

    pub const fn establishes_valence(&self) -> bool {
        false
    }

    pub const fn establishes_moral_patienthood(&self) -> bool {
        false
    }

    pub const fn establishes_metaphysical_identity(&self) -> bool {
        false
    }

    pub const fn grants_welfare_protection(&self) -> bool {
        false
    }

    pub const fn grants_legal_standing(&self) -> bool {
        false
    }

    pub const fn creates_civic_identity(&self) -> bool {
        false
    }

    pub const fn grants_currentness(&self) -> bool {
        false
    }

    pub const fn grants_deployment_authority(&self) -> bool {
        false
    }

    pub const fn grants_governance_authority(&self) -> bool {
        false
    }

    pub const fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn validate_candidate_subject_boundary(
    boundary: CandidateSubjectBoundary,
) -> Result<ValidatedCandidateSubjectBoundary, TopologyError> {
    validate_components(&boundary)?;
    validate_edges(&boundary)?;
    validate_alternatives(&boundary)?;
    validate_anchor(&boundary)?;
    validate_candidate_kind(&boundary)?;

    Ok(ValidatedCandidateSubjectBoundary { boundary })
}

pub fn validate_same_boundary_identity(
    left: &ValidatedCandidateSubjectBoundary,
    right: &ValidatedCandidateSubjectBoundary,
) -> Result<(), TopologyError> {
    if left.boundary.boundary_id != right.boundary.boundary_id {
        return Ok(());
    }

    if left.boundary != right.boundary {
        return Err(TopologyError::BoundaryEquivocation);
    }

    Ok(())
}

fn validate_components(boundary: &CandidateSubjectBoundary) -> Result<(), TopologyError> {
    if boundary.components.is_empty() {
        return Err(TopologyError::EmptyComponentSet);
    }
    if boundary.components.len() > MAX_COMPONENTS {
        return Err(TopologyError::TooManyComponents);
    }

    let mut component_ids = BTreeSet::new();
    let mut operational_subjects = BTreeSet::new();

    for node in &boundary.components {
        if !component_ids.insert(node.component_id) {
            return Err(TopologyError::DuplicateComponentId);
        }

        if let ComponentRef::Operational(subject) = node.component_ref {
            let key = (
                *subject.model_id.as_bytes(),
                *subject.lineage_id.as_bytes(),
                *subject.instance_id.as_bytes(),
            );
            if !operational_subjects.insert(key) {
                return Err(TopologyError::DuplicateOperationalComponent);
            }
        }
    }

    if boundary
        .components
        .windows(2)
        .any(|pair| pair[0].component_id > pair[1].component_id)
    {
        return Err(TopologyError::NonCanonicalComponentOrder);
    }

    Ok(())
}

fn validate_edges(boundary: &CandidateSubjectBoundary) -> Result<(), TopologyError> {
    if boundary.edges.len() > MAX_EDGES {
        return Err(TopologyError::TooManyEdges);
    }

    let component_ids: BTreeSet<ComponentId> = boundary
        .components
        .iter()
        .map(|node| node.component_id)
        .collect();
    let mut edge_keys = BTreeSet::new();

    for edge in &boundary.edges {
        if !component_ids.contains(&edge.from) || !component_ids.contains(&edge.to) {
            return Err(TopologyError::DanglingEdgeEndpoint);
        }
        if edge.from == edge.to {
            return Err(TopologyError::SelfEdge);
        }

        let key = (edge.from, edge.to, edge.kind);
        if !edge_keys.insert(key) {
            return Err(TopologyError::DuplicateEdge);
        }
    }

    if boundary.edges.windows(2).any(|pair| {
        (pair[0].from, pair[0].to, pair[0].kind) > (pair[1].from, pair[1].to, pair[1].kind)
    }) {
        return Err(TopologyError::NonCanonicalEdgeOrder);
    }

    Ok(())
}

fn validate_alternatives(boundary: &CandidateSubjectBoundary) -> Result<(), TopologyError> {
    if boundary.preregistered_alternatives.len() > MAX_ALTERNATIVES {
        return Err(TopologyError::TooManyAlternatives);
    }

    let mut hypotheses = BTreeSet::new();
    let mut boundaries = BTreeSet::new();

    for alternative in &boundary.preregistered_alternatives {
        if alternative.hypothesis_id == boundary.individuation_hypothesis_id {
            return Err(TopologyError::AlternativeReusesCurrentHypothesis);
        }
        if alternative.boundary_id == boundary.boundary_id {
            return Err(TopologyError::AlternativeReusesCurrentBoundary);
        }
        if !hypotheses.insert(alternative.hypothesis_id) {
            return Err(TopologyError::DuplicateAlternativeHypothesis);
        }
        if !boundaries.insert(alternative.boundary_id) {
            return Err(TopologyError::DuplicateAlternativeBoundary);
        }
    }

    if boundary
        .preregistered_alternatives
        .windows(2)
        .any(|pair| pair[0].hypothesis_id > pair[1].hypothesis_id)
    {
        return Err(TopologyError::NonCanonicalAlternativeOrder);
    }

    Ok(())
}

fn validate_anchor(boundary: &CandidateSubjectBoundary) -> Result<(), TopologyError> {
    let Some(anchor) = boundary.anchor_subject else {
        return Ok(());
    };

    let anchor_present = boundary.components.iter().any(|node| {
        matches!(node.component_ref, ComponentRef::Operational(subject) if subject == anchor)
    });

    if anchor_present {
        Ok(())
    } else {
        Err(TopologyError::AnchorMissingFromGraph)
    }
}

fn validate_candidate_kind(boundary: &CandidateSubjectBoundary) -> Result<(), TopologyError> {
    if boundary.candidate_kind != CandidateKind::SingleRuntimeInstance {
        return Ok(());
    }

    if boundary.components.len() != 1 {
        return Err(TopologyError::InvalidSingleRuntimeCardinality);
    }

    let ComponentRef::Operational(subject) = boundary.components[0].component_ref else {
        return Err(TopologyError::SingleRuntimeComponentNotOperational);
    };

    if boundary.anchor_subject != Some(subject) {
        return Err(TopologyError::SingleRuntimeAnchorMismatch);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use mycelix_ai_lineage::{InstanceId, LineageId, ModelId};

    fn bytes(value: u8) -> [u8; 32] {
        [value; 32]
    }

    fn boundary_id(value: u8) -> SubjectBoundaryId {
        SubjectBoundaryId::new(bytes(value)).expect("boundary id")
    }

    fn component_id(value: u8) -> ComponentId {
        ComponentId::new(bytes(value)).expect("component id")
    }

    fn hypothesis_id(value: u8) -> IndividuationHypothesisId {
        IndividuationHypothesisId::new(bytes(value)).expect("hypothesis id")
    }

    fn manifest(value: u8) -> TopologyManifestCommitment {
        TopologyManifestCommitment::new(bytes(value)).expect("manifest commitment")
    }

    fn epoch(value: u64) -> TopologyEpoch {
        TopologyEpoch::new(value).expect("epoch")
    }

    fn operational(model: u8, lineage: u8, instance: u8) -> OperationalSubjectRef {
        OperationalSubjectRef {
            model_id: ModelId::new(bytes(model)).expect("model"),
            lineage_id: LineageId::new(bytes(lineage)).expect("lineage"),
            instance_id: InstanceId::new(bytes(instance)).expect("instance"),
        }
    }

    fn operational_node(id: u8, subject: OperationalSubjectRef) -> ComponentNode {
        ComponentNode {
            component_id: component_id(id),
            component_ref: ComponentRef::Operational(subject),
        }
    }

    fn valid_single() -> CandidateSubjectBoundary {
        let subject = operational(1, 2, 3);
        CandidateSubjectBoundary {
            boundary_id: boundary_id(10),
            individuation_hypothesis_id: hypothesis_id(11),
            candidate_kind: CandidateKind::SingleRuntimeInstance,
            epoch: epoch(1),
            membership_policy: MembershipPolicy::Fixed,
            anchor_subject: Some(subject),
            topology_manifest_commitment: manifest(12),
            components: vec![operational_node(20, subject)],
            edges: vec![],
            preregistered_alternatives: vec![],
        }
    }

    fn valid_composite() -> CandidateSubjectBoundary {
        let left = operational(1, 2, 3);
        let right = operational(1, 4, 5);
        CandidateSubjectBoundary {
            boundary_id: boundary_id(30),
            individuation_hypothesis_id: hypothesis_id(31),
            candidate_kind: CandidateKind::CompositeAgent,
            epoch: epoch(2),
            membership_policy: MembershipPolicy::Dynamic,
            anchor_subject: Some(left),
            topology_manifest_commitment: manifest(32),
            components: vec![operational_node(40, left), operational_node(41, right)],
            edges: vec![CompositionEdge {
                from: component_id(40),
                to: component_id(41),
                kind: CompositionEdgeKind::Control,
            }],
            preregistered_alternatives: vec![AlternativeBoundaryRef {
                hypothesis_id: hypothesis_id(50),
                boundary_id: boundary_id(51),
            }],
        }
    }

    #[test]
    fn identifiers_and_epochs_reject_zero() {
        assert_eq!(
            SubjectBoundaryId::new([0; 32]),
            Err(TopologyError::ZeroIdentifier(
                IdentifierKind::SubjectBoundary
            ))
        );
        assert_eq!(TopologyEpoch::new(0), Err(TopologyError::ZeroEpoch));
    }

    #[test]
    fn single_runtime_boundary_requires_one_anchored_operational_component() {
        let qualified =
            validate_candidate_subject_boundary(valid_single()).expect("valid single boundary");
        assert_eq!(qualified.component_count(), 1);
        assert_eq!(qualified.operational_component_count(), 1);

        let mut invalid = valid_single();
        invalid.components.push(operational_node(21, operational(1, 4, 5)));
        assert_eq!(
            validate_candidate_subject_boundary(invalid),
            Err(TopologyError::InvalidSingleRuntimeCardinality)
        );
    }

    #[test]
    fn duplicate_component_ids_are_rejected() {
        let mut boundary = valid_composite();
        boundary.components[1].component_id = boundary.components[0].component_id;
        assert_eq!(
            validate_candidate_subject_boundary(boundary),
            Err(TopologyError::DuplicateComponentId)
        );
    }

    #[test]
    fn duplicate_operational_components_are_rejected() {
        let mut boundary = valid_composite();
        boundary.components[1].component_ref = boundary.components[0].component_ref;
        assert_eq!(
            validate_candidate_subject_boundary(boundary),
            Err(TopologyError::DuplicateOperationalComponent)
        );
    }

    #[test]
    fn noncanonical_component_order_is_rejected() {
        let mut boundary = valid_composite();
        boundary.components.swap(0, 1);
        assert_eq!(
            validate_candidate_subject_boundary(boundary),
            Err(TopologyError::NonCanonicalComponentOrder)
        );
    }

    #[test]
    fn dangling_self_and_duplicate_edges_are_rejected() {
        let mut dangling = valid_composite();
        dangling.edges[0].to = component_id(99);
        assert_eq!(
            validate_candidate_subject_boundary(dangling),
            Err(TopologyError::DanglingEdgeEndpoint)
        );

        let mut self_edge = valid_composite();
        self_edge.edges[0].to = self_edge.edges[0].from;
        assert_eq!(
            validate_candidate_subject_boundary(self_edge),
            Err(TopologyError::SelfEdge)
        );

        let mut duplicate = valid_composite();
        duplicate.edges.push(duplicate.edges[0]);
        assert_eq!(
            validate_candidate_subject_boundary(duplicate),
            Err(TopologyError::DuplicateEdge)
        );
    }

    #[test]
    fn anchor_must_be_present_in_the_graph() {
        let mut boundary = valid_composite();
        boundary.anchor_subject = Some(operational(9, 8, 7));
        assert_eq!(
            validate_candidate_subject_boundary(boundary),
            Err(TopologyError::AnchorMissingFromGraph)
        );
    }

    #[test]
    fn alternative_hypotheses_and_boundaries_must_be_distinct() {
        let mut same_hypothesis = valid_composite();
        same_hypothesis.preregistered_alternatives[0].hypothesis_id =
            same_hypothesis.individuation_hypothesis_id;
        assert_eq!(
            validate_candidate_subject_boundary(same_hypothesis),
            Err(TopologyError::AlternativeReusesCurrentHypothesis)
        );

        let mut same_boundary = valid_composite();
        same_boundary.preregistered_alternatives[0].boundary_id = same_boundary.boundary_id;
        assert_eq!(
            validate_candidate_subject_boundary(same_boundary),
            Err(TopologyError::AlternativeReusesCurrentBoundary)
        );
    }

    #[test]
    fn service_pool_topology_records_operational_multiplicity_only() {
        let mut boundary = valid_composite();
        boundary.candidate_kind = CandidateKind::ServicePool;
        let qualified =
            validate_candidate_subject_boundary(boundary).expect("valid service pool");
        assert_eq!(qualified.operational_component_count(), 2);
        assert!(!qualified.establishes_moral_patienthood());
        assert!(!qualified.establishes_metaphysical_identity());
    }

    #[test]
    fn same_boundary_id_with_changed_manifest_is_equivocation() {
        let left = validate_candidate_subject_boundary(valid_composite()).expect("left boundary");
        let mut changed = valid_composite();
        changed.topology_manifest_commitment = manifest(90);
        let right = validate_candidate_subject_boundary(changed).expect("right boundary");
        assert_eq!(
            validate_same_boundary_identity(&left, &right),
            Err(TopologyError::BoundaryEquivocation)
        );
    }

    #[test]
    fn new_boundary_id_allows_a_new_topology_epoch() {
        let left = validate_candidate_subject_boundary(valid_composite()).expect("left boundary");
        let mut changed = valid_composite();
        changed.boundary_id = boundary_id(91);
        changed.epoch = epoch(3);
        changed.topology_manifest_commitment = manifest(92);
        let right = validate_candidate_subject_boundary(changed).expect("right boundary");
        assert!(validate_same_boundary_identity(&left, &right).is_ok());
    }

    #[test]
    fn validated_topology_grants_no_status_or_authority() {
        let qualified =
            validate_candidate_subject_boundary(valid_composite()).expect("qualified topology");
        assert!(!qualified.establishes_consciousness());
        assert!(!qualified.establishes_valence());
        assert!(!qualified.establishes_moral_patienthood());
        assert!(!qualified.establishes_metaphysical_identity());
        assert!(!qualified.grants_welfare_protection());
        assert!(!qualified.grants_legal_standing());
        assert!(!qualified.creates_civic_identity());
        assert!(!qualified.grants_currentness());
        assert!(!qualified.grants_deployment_authority());
        assert!(!qualified.grants_governance_authority());
        assert!(!qualified.grants_external_effect_authority());
    }
}
