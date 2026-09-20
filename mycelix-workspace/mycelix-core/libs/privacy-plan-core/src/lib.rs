// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Authority-neutral privacy composition plans layered over Mycelix PEC.
//!
//! This crate validates plan structure only. It does not authenticate evidence,
//! prove privacy, select cryptographic backends, or grant application authority.

use privacy_computation_core::{PrimitiveCapability, QualificationState, SemanticAuthority};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PlanStepId(pub u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PlanIntent {
    Exploratory,
    Production,
}

/// Reference to evidence that another verifier is expected to authenticate.
/// Presence is not authentication.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceReference {
    pub receipt_digest: String,
    pub verifier_profile: String,
}

impl EvidenceReference {
    pub fn is_structurally_complete(&self) -> bool {
        !self.receipt_digest.trim().is_empty() && !self.verifier_profile.trim().is_empty()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ArtifactClass {
    NoCollectedData,
    LocalValue,
    Plaintext,
    Ciphertext,
    Proof,
    PrivateSetResult,
    Aggregate,
    QueryResponse,
    StatisticalRelease,
    OpaqueQualified,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlanStepKind {
    DoNotCollect,
    LocalOnlyComputation { computation_id: String },
    OrdinaryEncryption { profile_id: String },
    QualifiedPrimitive {
        capability: PrimitiveCapability,
        qualification_evidence: Option<EvidenceReference>,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanStep {
    pub id: PlanStepId,
    pub kind: PlanStepKind,
    pub accepts: Vec<ArtifactClass>,
    pub produces: Vec<ArtifactClass>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PlanEdge {
    pub from: PlanStepId,
    pub to: PlanStepId,
    pub artifact: ArtifactClass,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrivacyPlan {
    pub requirement_profile: String,
    pub intent: PlanIntent,
    pub steps: Vec<PlanStep>,
    pub edges: Vec<PlanEdge>,
    /// Required structurally for multi-step production plans. This is still only
    /// an evidence reference, not authenticated composition evidence.
    pub composition_evidence: Option<EvidenceReference>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PlanFailure {
    MissingRequirementProfile,
    EmptyPlan,
    DuplicateStepId,
    MissingLocalComputationIdentity,
    MissingEncryptionProfile,
    ProductionPrimitiveNotAdmitted,
    MissingPrimitiveEvidence,
    DoNotCollectMustBeStandalone,
    EdgeReferencesUnknownStep,
    SelfEdge,
    ProducerDoesNotEmitArtifact,
    ConsumerDoesNotAcceptArtifact,
    CyclicDataflow,
    MissingCompositionEvidence,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PlanDisposition {
    StructurallyCompatible,
    Incompatible(PlanFailure),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanEvaluation {
    pub disposition: PlanDisposition,
    pub authority: SemanticAuthority,
}

impl PlanEvaluation {
    fn compatible() -> Self {
        Self {
            disposition: PlanDisposition::StructurallyCompatible,
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    fn incompatible(reason: PlanFailure) -> Self {
        Self {
            disposition: PlanDisposition::Incompatible(reason),
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    pub const fn privacy_established(&self) -> bool {
        false
    }

    pub const fn evidence_authenticated(&self) -> bool {
        false
    }

    pub const fn composition_qualified(&self) -> bool {
        false
    }

    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NoQualifiedPlan {
    pub requirement_profile: String,
    pub authority: SemanticAuthority,
}

impl NoQualifiedPlan {
    pub fn new(requirement_profile: impl Into<String>) -> Self {
        Self {
            requirement_profile: requirement_profile.into(),
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    pub const fn permission_to_weaken_requirement(&self) -> bool {
        false
    }

    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

fn complete_evidence(reference: &Option<EvidenceReference>) -> bool {
    reference
        .as_ref()
        .is_some_and(EvidenceReference::is_structurally_complete)
}

fn dataflow_is_acyclic(plan: &PrivacyPlan, step_map: &HashMap<PlanStepId, &PlanStep>) -> bool {
    let mut indegree: HashMap<PlanStepId, usize> =
        step_map.keys().copied().map(|id| (id, 0)).collect();
    let mut outgoing: HashMap<PlanStepId, Vec<PlanStepId>> = HashMap::new();

    for edge in &plan.edges {
        *indegree.entry(edge.to).or_insert(0) += 1;
        outgoing.entry(edge.from).or_default().push(edge.to);
    }

    let mut queue: VecDeque<PlanStepId> = indegree
        .iter()
        .filter_map(|(id, degree)| (*degree == 0).then_some(*id))
        .collect();
    let mut visited = 0usize;

    while let Some(id) = queue.pop_front() {
        visited += 1;
        if let Some(targets) = outgoing.get(&id) {
            for target in targets {
                let degree = indegree
                    .get_mut(target)
                    .expect("edge targets were validated before cycle checking");
                *degree -= 1;
                if *degree == 0 {
                    queue.push_back(*target);
                }
            }
        }
    }

    visited == step_map.len()
}

pub fn evaluate_plan(plan: &PrivacyPlan) -> PlanEvaluation {
    if plan.requirement_profile.trim().is_empty() {
        return PlanEvaluation::incompatible(PlanFailure::MissingRequirementProfile);
    }

    if plan.steps.is_empty() {
        return PlanEvaluation::incompatible(PlanFailure::EmptyPlan);
    }

    let mut step_map = HashMap::new();
    for step in &plan.steps {
        if step_map.insert(step.id, step).is_some() {
            return PlanEvaluation::incompatible(PlanFailure::DuplicateStepId);
        }

        match &step.kind {
            PlanStepKind::DoNotCollect => {}
            PlanStepKind::LocalOnlyComputation { computation_id } => {
                if computation_id.trim().is_empty() {
                    return PlanEvaluation::incompatible(
                        PlanFailure::MissingLocalComputationIdentity,
                    );
                }
            }
            PlanStepKind::OrdinaryEncryption { profile_id } => {
                if profile_id.trim().is_empty() {
                    return PlanEvaluation::incompatible(PlanFailure::MissingEncryptionProfile);
                }
            }
            PlanStepKind::QualifiedPrimitive {
                capability,
                qualification_evidence,
            } => {
                if matches!(plan.intent, PlanIntent::Production) {
                    if capability.qualification != QualificationState::ProductionAdmitted {
                        return PlanEvaluation::incompatible(
                            PlanFailure::ProductionPrimitiveNotAdmitted,
                        );
                    }
                    if !complete_evidence(qualification_evidence) {
                        return PlanEvaluation::incompatible(PlanFailure::MissingPrimitiveEvidence);
                    }
                }
            }
        }
    }

    let do_not_collect_count = plan
        .steps
        .iter()
        .filter(|step| matches!(step.kind, PlanStepKind::DoNotCollect))
        .count();
    if do_not_collect_count > 0 && (plan.steps.len() != 1 || !plan.edges.is_empty()) {
        return PlanEvaluation::incompatible(PlanFailure::DoNotCollectMustBeStandalone);
    }

    for edge in &plan.edges {
        let Some(producer) = step_map.get(&edge.from) else {
            return PlanEvaluation::incompatible(PlanFailure::EdgeReferencesUnknownStep);
        };
        let Some(consumer) = step_map.get(&edge.to) else {
            return PlanEvaluation::incompatible(PlanFailure::EdgeReferencesUnknownStep);
        };

        if edge.from == edge.to {
            return PlanEvaluation::incompatible(PlanFailure::SelfEdge);
        }
        if !producer.produces.contains(&edge.artifact) {
            return PlanEvaluation::incompatible(PlanFailure::ProducerDoesNotEmitArtifact);
        }
        if !consumer.accepts.contains(&edge.artifact) {
            return PlanEvaluation::incompatible(PlanFailure::ConsumerDoesNotAcceptArtifact);
        }
    }

    if !dataflow_is_acyclic(plan, &step_map) {
        return PlanEvaluation::incompatible(PlanFailure::CyclicDataflow);
    }

    if matches!(plan.intent, PlanIntent::Production)
        && plan.steps.len() > 1
        && !complete_evidence(&plan.composition_evidence)
    {
        return PlanEvaluation::incompatible(PlanFailure::MissingCompositionEvidence);
    }

    PlanEvaluation::compatible()
}

#[cfg(test)]
mod tests {
    use super::*;
    use privacy_computation_core::{
        AdversaryModel, BackendIdentity, InteractionModel, LeakageProfile, ParticipantModel,
        PrivacyObjective, PrivacyPrimitive,
    };

    fn capability(qualification: QualificationState) -> PrimitiveCapability {
        PrimitiveCapability {
            backend: BackendIdentity {
                primitive: PrivacyPrimitive::MultiPartyComputation,
                backend: "synthetic".into(),
                version: "0".into(),
                profile: "test".into(),
            },
            supported_objectives: vec![PrivacyObjective::JointInputConfidentiality],
            participant_model: ParticipantModel::MultiParty { participants: 3 },
            adversary_model: AdversaryModel::HonestButCurious,
            interaction_model: InteractionModel::Interactive { rounds: 2 },
            leakage: LeakageProfile::default(),
            qualification,
        }
    }

    fn evidence() -> EvidenceReference {
        EvidenceReference {
            receipt_digest: "sha256:deadbeef".into(),
            verifier_profile: "qualified-receipt-v1".into(),
        }
    }

    fn primitive_step(id: u32, qualification: QualificationState) -> PlanStep {
        PlanStep {
            id: PlanStepId(id),
            kind: PlanStepKind::QualifiedPrimitive {
                capability: capability(qualification),
                qualification_evidence: Some(evidence()),
            },
            accepts: vec![ArtifactClass::Plaintext, ArtifactClass::Ciphertext],
            produces: vec![ArtifactClass::Aggregate, ArtifactClass::Ciphertext],
        }
    }

    #[test]
    fn production_primitive_requires_production_admission() {
        let plan = PrivacyPlan {
            requirement_profile: "req-v1".into(),
            intent: PlanIntent::Production,
            steps: vec![primitive_step(1, QualificationState::Qualified)],
            edges: vec![],
            composition_evidence: None,
        };
        assert_eq!(
            evaluate_plan(&plan).disposition,
            PlanDisposition::Incompatible(PlanFailure::ProductionPrimitiveNotAdmitted)
        );
    }

    #[test]
    fn production_primitive_requires_evidence_reference() {
        let mut step = primitive_step(1, QualificationState::ProductionAdmitted);
        if let PlanStepKind::QualifiedPrimitive {
            qualification_evidence,
            ..
        } = &mut step.kind
        {
            *qualification_evidence = None;
        }
        let plan = PrivacyPlan {
            requirement_profile: "req-v1".into(),
            intent: PlanIntent::Production,
            steps: vec![step],
            edges: vec![],
            composition_evidence: None,
        };
        assert_eq!(
            evaluate_plan(&plan).disposition,
            PlanDisposition::Incompatible(PlanFailure::MissingPrimitiveEvidence)
        );
    }

    #[test]
    fn individually_admitted_steps_do_not_admit_composition() {
        let mut a = primitive_step(1, QualificationState::ProductionAdmitted);
        a.produces = vec![ArtifactClass::Ciphertext];
        let mut b = primitive_step(2, QualificationState::ProductionAdmitted);
        b.accepts = vec![ArtifactClass::Ciphertext];
        let plan = PrivacyPlan {
            requirement_profile: "req-v1".into(),
            intent: PlanIntent::Production,
            steps: vec![a, b],
            edges: vec![PlanEdge {
                from: PlanStepId(1),
                to: PlanStepId(2),
                artifact: ArtifactClass::Ciphertext,
            }],
            composition_evidence: None,
        };
        assert_eq!(
            evaluate_plan(&plan).disposition,
            PlanDisposition::Incompatible(PlanFailure::MissingCompositionEvidence)
        );
    }

    #[test]
    fn artifact_mismatch_fails_closed() {
        let mut a = primitive_step(1, QualificationState::Experimental);
        a.produces = vec![ArtifactClass::Proof];
        let mut b = primitive_step(2, QualificationState::Experimental);
        b.accepts = vec![ArtifactClass::Ciphertext];
        let plan = PrivacyPlan {
            requirement_profile: "req-v1".into(),
            intent: PlanIntent::Exploratory,
            steps: vec![a, b],
            edges: vec![PlanEdge {
                from: PlanStepId(1),
                to: PlanStepId(2),
                artifact: ArtifactClass::Proof,
            }],
            composition_evidence: None,
        };
        assert_eq!(
            evaluate_plan(&plan).disposition,
            PlanDisposition::Incompatible(PlanFailure::ConsumerDoesNotAcceptArtifact)
        );
    }

    #[test]
    fn cycles_fail_closed() {
        let mut a = primitive_step(1, QualificationState::Experimental);
        a.accepts = vec![ArtifactClass::Ciphertext];
        a.produces = vec![ArtifactClass::Ciphertext];
        let mut b = primitive_step(2, QualificationState::Experimental);
        b.accepts = vec![ArtifactClass::Ciphertext];
        b.produces = vec![ArtifactClass::Ciphertext];
        let plan = PrivacyPlan {
            requirement_profile: "req-v1".into(),
            intent: PlanIntent::Exploratory,
            steps: vec![a, b],
            edges: vec![
                PlanEdge {
                    from: PlanStepId(1),
                    to: PlanStepId(2),
                    artifact: ArtifactClass::Ciphertext,
                },
                PlanEdge {
                    from: PlanStepId(2),
                    to: PlanStepId(1),
                    artifact: ArtifactClass::Ciphertext,
                },
            ],
            composition_evidence: None,
        };
        assert_eq!(
            evaluate_plan(&plan).disposition,
            PlanDisposition::Incompatible(PlanFailure::CyclicDataflow)
        );
    }

    #[test]
    fn do_not_collect_is_first_class_but_non_authoritative() {
        let plan = PrivacyPlan {
            requirement_profile: "req-v1".into(),
            intent: PlanIntent::Exploratory,
            steps: vec![PlanStep {
                id: PlanStepId(1),
                kind: PlanStepKind::DoNotCollect,
                accepts: vec![],
                produces: vec![ArtifactClass::NoCollectedData],
            }],
            edges: vec![],
            composition_evidence: None,
        };
        let result = evaluate_plan(&plan);
        assert_eq!(result.disposition, PlanDisposition::StructurallyCompatible);
        assert_eq!(result.authority, SemanticAuthority::StructuralOnly);
        assert!(!result.privacy_established());
        assert!(!result.evidence_authenticated());
        assert!(!result.composition_qualified());
        assert!(!result.application_authority_granted());
    }

    #[test]
    fn no_qualified_plan_never_permits_requirement_weakening() {
        let no_plan = NoQualifiedPlan::new("req-v1");
        assert_eq!(no_plan.authority, SemanticAuthority::StructuralOnly);
        assert!(!no_plan.permission_to_weaken_requirement());
        assert!(!no_plan.application_authority_granted());
    }

    #[test]
    fn ordinary_encryption_name_does_not_mint_privacy() {
        let plan = PrivacyPlan {
            requirement_profile: "req-v1".into(),
            intent: PlanIntent::Exploratory,
            steps: vec![PlanStep {
                id: PlanStepId(1),
                kind: PlanStepKind::OrdinaryEncryption {
                    profile_id: "super-private-encryption".into(),
                },
                accepts: vec![ArtifactClass::Plaintext],
                produces: vec![ArtifactClass::Ciphertext],
            }],
            edges: vec![],
            composition_evidence: None,
        };
        let result = evaluate_plan(&plan);
        assert_eq!(result.disposition, PlanDisposition::StructurallyCompatible);
        assert!(!result.privacy_established());
        assert!(!result.application_authority_granted());
    }
}
