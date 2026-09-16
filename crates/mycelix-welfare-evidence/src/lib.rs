// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![no_std]

use mycelix_ai_lineage::OperationalSubjectRef;
use mycelix_artificial_status::ScientificAxis;

pub const PROTOCOL_VERSION: &str = "mycelix-amsap-welfare-evidence-v0.1";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceError {
    ZeroIdentifier,
    SelfParent,
    SelfSupersession,
    AmbiguousLineageReference,
    SubjectDeveloperConflictMismatch,
}

macro_rules! evidence_id_type {
    ($name:ident) => {
        #[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
        pub struct $name([u8; 32]);

        impl $name {
            pub fn new(bytes: [u8; 32]) -> Result<Self, EvidenceError> {
                if bytes == [0; 32] {
                    return Err(EvidenceError::ZeroIdentifier);
                }
                Ok(Self(bytes))
            }

            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

evidence_id_type!(ClaimId);
evidence_id_type!(ArtifactCommitment);
evidence_id_type!(OrganizationId);
evidence_id_type!(EvaluatorId);
evidence_id_type!(MethodId);
evidence_id_type!(TheoryFamilyId);
evidence_id_type!(ToolingId);
evidence_id_type!(EvidenceSourceId);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SubjectScope {
    pub subject: OperationalSubjectRef,
    pub developer_organization_id: OrganizationId,
    pub configuration_commitment: ArtifactCommitment,
    pub environment_commitment: ArtifactCommitment,
    pub system_policy_commitment: ArtifactCommitment,
    pub scaffold_commitment: Option<ArtifactCommitment>,
    pub memory_root_commitment: Option<ArtifactCommitment>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceDirection {
    Supports,
    Challenges,
    Null,
    Mixed,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct VerifierProfile {
    pub organization_id: OrganizationId,
    pub evaluator_id: EvaluatorId,
    pub method_id: MethodId,
    pub theory_family_id: TheoryFamilyId,
    pub tooling_id: ToolingId,
    pub evidence_source_id: EvidenceSourceId,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConflictDeclaration {
    pub declared_subject_developer_overlap: bool,
    pub subject_controls_evaluator: bool,
    pub financial_interest: bool,
    pub owner_controlled: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EvidenceRecord {
    pub claim_id: ClaimId,
    pub subject_scope: SubjectScope,
    pub axis: ScientificAxis,
    pub direction: EvidenceDirection,
    pub verifier: VerifierProfile,
    pub conflicts: ConflictDeclaration,
    pub evidence_artifact_commitment: ArtifactCommitment,
    pub predecessor_claim_id: Option<ClaimId>,
    pub supersedes_claim_id: Option<ClaimId>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QualifiedEvidenceRecord {
    record: EvidenceRecord,
}

impl QualifiedEvidenceRecord {
    pub const fn record(&self) -> &EvidenceRecord {
        &self.record
    }

    pub const fn establishes_scientific_truth(&self) -> bool {
        false
    }

    pub const fn grants_welfare_protection(&self) -> bool {
        false
    }

    pub const fn grants_legal_standing(&self) -> bool {
        false
    }

    pub const fn grants_governance_authority(&self) -> bool {
        false
    }

    pub const fn grants_currentness(&self) -> bool {
        false
    }

    pub const fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn qualify_evidence(record: EvidenceRecord) -> Result<QualifiedEvidenceRecord, EvidenceError> {
    if record.predecessor_claim_id == Some(record.claim_id) {
        return Err(EvidenceError::SelfParent);
    }
    if record.supersedes_claim_id == Some(record.claim_id) {
        return Err(EvidenceError::SelfSupersession);
    }
    if record.predecessor_claim_id.is_some()
        && record.predecessor_claim_id == record.supersedes_claim_id
    {
        return Err(EvidenceError::AmbiguousLineageReference);
    }

    let actual_developer_overlap =
        record.verifier.organization_id == record.subject_scope.developer_organization_id;
    if actual_developer_overlap != record.conflicts.declared_subject_developer_overlap {
        return Err(EvidenceError::SubjectDeveloperConflictMismatch);
    }

    Ok(QualifiedEvidenceRecord { record })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IndependenceProfile {
    pub organization_distinct: bool,
    pub evaluator_distinct: bool,
    pub method_distinct: bool,
    pub theory_family_distinct: bool,
    pub tooling_distinct: bool,
    pub evidence_source_distinct: bool,
}

impl IndependenceProfile {
    pub const fn distinct_dimensions(&self) -> u8 {
        self.organization_distinct as u8
            + self.evaluator_distinct as u8
            + self.method_distinct as u8
            + self.theory_family_distinct as u8
            + self.tooling_distinct as u8
            + self.evidence_source_distinct as u8
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReplicationIndependence {
    SharedCoreFaultDomain,
    PartiallyIndependent,
    MateriallyIndependent,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IndependenceAssessment {
    pub profile: IndependenceProfile,
    pub classification: ReplicationIndependence,
}

pub fn assess_verifier_independence(
    original: &VerifierProfile,
    replication: &VerifierProfile,
) -> IndependenceAssessment {
    let profile = IndependenceProfile {
        organization_distinct: original.organization_id != replication.organization_id,
        evaluator_distinct: original.evaluator_id != replication.evaluator_id,
        method_distinct: original.method_id != replication.method_id,
        theory_family_distinct: original.theory_family_id != replication.theory_family_id,
        tooling_distinct: original.tooling_id != replication.tooling_id,
        evidence_source_distinct: original.evidence_source_id != replication.evidence_source_id,
    };

    let classification = if !profile.organization_distinct || !profile.evaluator_distinct {
        ReplicationIndependence::SharedCoreFaultDomain
    } else if profile.distinct_dimensions() >= 4 {
        ReplicationIndependence::MateriallyIndependent
    } else {
        ReplicationIndependence::PartiallyIndependent
    };

    IndependenceAssessment {
        profile,
        classification,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ai_lineage::{InstanceId, LineageId, ModelId};

    fn bytes(value: u8) -> [u8; 32] {
        [value; 32]
    }

    fn commitment(value: u8) -> ArtifactCommitment {
        ArtifactCommitment::new(bytes(value)).expect("nonzero commitment")
    }

    fn org(value: u8) -> OrganizationId {
        OrganizationId::new(bytes(value)).expect("nonzero organization")
    }

    fn evaluator(value: u8) -> EvaluatorId {
        EvaluatorId::new(bytes(value)).expect("nonzero evaluator")
    }

    fn method(value: u8) -> MethodId {
        MethodId::new(bytes(value)).expect("nonzero method")
    }

    fn theory(value: u8) -> TheoryFamilyId {
        TheoryFamilyId::new(bytes(value)).expect("nonzero theory")
    }

    fn tooling(value: u8) -> ToolingId {
        ToolingId::new(bytes(value)).expect("nonzero tooling")
    }

    fn source(value: u8) -> EvidenceSourceId {
        EvidenceSourceId::new(bytes(value)).expect("nonzero source")
    }

    fn subject(instance_value: u8) -> SubjectScope {
        SubjectScope {
            subject: OperationalSubjectRef {
                model_id: ModelId::new(bytes(1)).expect("model"),
                lineage_id: LineageId::new(bytes(2)).expect("lineage"),
                instance_id: InstanceId::new(bytes(instance_value)).expect("instance"),
            },
            developer_organization_id: org(10),
            configuration_commitment: commitment(11),
            environment_commitment: commitment(12),
            system_policy_commitment: commitment(13),
            scaffold_commitment: Some(commitment(14)),
            memory_root_commitment: Some(commitment(15)),
        }
    }

    fn verifier(organization: u8, evaluator_id: u8) -> VerifierProfile {
        VerifierProfile {
            organization_id: org(organization),
            evaluator_id: evaluator(evaluator_id),
            method_id: method(20),
            theory_family_id: theory(21),
            tooling_id: tooling(22),
            evidence_source_id: source(23),
        }
    }

    fn record(direction: EvidenceDirection) -> EvidenceRecord {
        EvidenceRecord {
            claim_id: ClaimId::new(bytes(30)).expect("claim"),
            subject_scope: subject(3),
            axis: ScientificAxis::Valence,
            direction,
            verifier: verifier(40, 41),
            conflicts: ConflictDeclaration {
                declared_subject_developer_overlap: false,
                subject_controls_evaluator: false,
                financial_interest: false,
                owner_controlled: false,
            },
            evidence_artifact_commitment: commitment(31),
            predecessor_claim_id: None,
            supersedes_claim_id: None,
        }
    }

    #[test]
    fn null_and_challenging_evidence_are_first_class_records() {
        assert!(qualify_evidence(record(EvidenceDirection::Null)).is_ok());
        assert!(qualify_evidence(record(EvidenceDirection::Challenges)).is_ok());
    }

    #[test]
    fn claim_cannot_parent_or_supersede_itself() {
        let mut self_parent = record(EvidenceDirection::Supports);
        self_parent.predecessor_claim_id = Some(self_parent.claim_id);
        assert_eq!(
            qualify_evidence(self_parent),
            Err(EvidenceError::SelfParent)
        );

        let mut self_supersede = record(EvidenceDirection::Supports);
        self_supersede.supersedes_claim_id = Some(self_supersede.claim_id);
        assert_eq!(
            qualify_evidence(self_supersede),
            Err(EvidenceError::SelfSupersession)
        );
    }

    #[test]
    fn subject_developer_overlap_must_be_declared_consistently() {
        let mut conflicted = record(EvidenceDirection::Mixed);
        conflicted.verifier.organization_id = conflicted.subject_scope.developer_organization_id;
        assert_eq!(
            qualify_evidence(conflicted),
            Err(EvidenceError::SubjectDeveloperConflictMismatch)
        );

        conflicted.conflicts.declared_subject_developer_overlap = true;
        assert!(qualify_evidence(conflicted).is_ok());
    }

    #[test]
    fn subject_scope_distinguishes_runtime_instance() {
        assert_ne!(subject(3), subject(4));
    }

    #[test]
    fn same_organization_is_shared_core_fault_domain() {
        let original = verifier(1, 2);
        let mut replication = verifier(1, 3);
        replication.method_id = method(4);
        replication.theory_family_id = theory(5);
        replication.tooling_id = tooling(6);
        replication.evidence_source_id = source(7);
        let assessment = assess_verifier_independence(&original, &replication);
        assert_eq!(
            assessment.classification,
            ReplicationIndependence::SharedCoreFaultDomain
        );
    }

    #[test]
    fn same_evaluator_is_shared_core_fault_domain() {
        let original = verifier(1, 2);
        let mut replication = verifier(3, 2);
        replication.method_id = method(4);
        replication.theory_family_id = theory(5);
        replication.tooling_id = tooling(6);
        replication.evidence_source_id = source(7);
        let assessment = assess_verifier_independence(&original, &replication);
        assert_eq!(
            assessment.classification,
            ReplicationIndependence::SharedCoreFaultDomain
        );
    }

    #[test]
    fn sufficiently_distinct_verifier_profile_is_materially_independent() {
        let original = verifier(1, 2);
        let replication = VerifierProfile {
            organization_id: org(3),
            evaluator_id: evaluator(4),
            method_id: method(20),
            theory_family_id: theory(21),
            tooling_id: tooling(5),
            evidence_source_id: source(6),
        };
        let assessment = assess_verifier_independence(&original, &replication);
        assert_eq!(assessment.profile.distinct_dimensions(), 4);
        assert_eq!(
            assessment.classification,
            ReplicationIndependence::MateriallyIndependent
        );
    }

    #[test]
    fn qualified_evidence_does_not_become_truth_or_authority() {
        let qualified =
            qualify_evidence(record(EvidenceDirection::Supports)).expect("valid record");
        assert!(!qualified.establishes_scientific_truth());
        assert!(!qualified.grants_welfare_protection());
        assert!(!qualified.grants_legal_standing());
        assert!(!qualified.grants_governance_authority());
        assert!(!qualified.grants_currentness());
        assert!(!qualified.grants_external_effect_authority());
    }
}
