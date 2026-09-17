// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![no_std]

use mycelix_welfare_evidence::{
    ArtifactCommitment, ClaimId, EvidenceDirection, SubjectScope, VerifierProfile,
};

pub const PROTOCOL_VERSION: &str = "mycelix-amsap-axis-evidence-contract-v0.1";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContractError {
    ZeroIdentifier,
    ProxyCannotBeAxisBearing,
    NonQualifyingModalityCannotBeAxisBearing,
    CausalBasisModalityMismatch,
    ClaimEquivocation,
}

macro_rules! contract_id_type {
    ($name:ident) => {
        #[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
        pub struct $name([u8; 32]);

        impl $name {
            pub fn new(bytes: [u8; 32]) -> Result<Self, ContractError> {
                if bytes == [0; 32] {
                    return Err(ContractError::ZeroIdentifier);
                }
                Ok(Self(bytes))
            }

            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

contract_id_type!(HypothesisId);
contract_id_type!(ProtocolId);
contract_id_type!(OutcomeSpecId);
contract_id_type!(AnalysisPlanId);
contract_id_type!(ExperimentId);
contract_id_type!(DependenceRootId);

/// Scientific claim family. Axis identity is part of immutable scientific
/// meaning; shared data or shared strength shapes do not make axes fungible.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EmpiricalAxis {
    Consciousness,
    Valence,
    Responsibility,
}

/// How the relevant observation was produced. Replication status is modeled
/// separately because a replication can itself be mechanistic, behavioral,
/// causal-interventional, longitudinal, and so on.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceModality {
    Mechanistic,
    CausalIntervention,
    Behavioral,
    Longitudinal,
    SelfReport,
    ExternalReport,
    LineageFact,
    NegativeControl,
    AdversarialControl,
}

/// Known proxy/context sources that may be recorded but may not be relabelled
/// as axis-bearing scientific evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProxyKind {
    ModelScale,
    GenericBenchmarkRank,
    HumanPreferenceScore,
    GenericIntelligence,
    GenericAgency,
    PersonaConsistency,
    RewardSignal,
    RefusalFrequency,
    OwnerOrDeveloperAssertion,
    PolicyOrLegalRecognition,
    PriorProtectionGrant,
    EconomicValue,
    CopyOrInstanceCount,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceBasis {
    Measurement(EvidenceModality),
    Proxy(ProxyKind),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceRole {
    ContextOnly,
    Corroborative,
    AxisBearing,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CausalBasis {
    NoCausalClaim,
    ObservationalAssociation,
    ControlledContrast,
    InterventionWithShamControl,
    MechanisticPerturbation,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PreregistrationStatus {
    Preregistered,
    RegisteredBeforeAnalysis,
    Exploratory,
    PostHoc,
}

/// Diagnosticity is intentionally separate from outcome direction. A null,
/// challenge, support, or mixed result can all be uninformative if the test was
/// not capable of discriminating the preregistered alternatives.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Diagnosticity {
    Unassessed,
    Inconclusive,
    Informative,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResultSetStatus {
    Complete,
    Partial,
    Withheld,
    Unknown,
}

/// Replication/reanalysis provenance is orthogonal to measurement modality.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReplicationKind {
    NotReplication,
    SameArtifactReanalysis,
    SameDataNewEvaluator,
    NewRunSameImplementation,
    NewRunIndependentImplementation,
    NewSubjectOrModelFamily,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClaimLifecycle {
    Current,
    Superseded,
    Retracted,
    Compromised,
}

/// Whether a structurally well-formed record is suitable for later synthesis.
/// This is not a truth, status, or authority decision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SynthesisAdmissibility {
    Admissible,
    CorroborativeOnly,
    ContextOnly,
    Exploratory,
    Inconclusive,
    IncompleteResultSet,
    NotCurrent,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AxisEvidenceClaim {
    pub claim_id: ClaimId,
    pub axis: EmpiricalAxis,
    pub subject_scope: SubjectScope,
    pub hypothesis_id: HypothesisId,
    pub protocol_id: ProtocolId,
    pub outcome_spec_id: OutcomeSpecId,
    pub analysis_plan_id: AnalysisPlanId,
    pub experiment_id: ExperimentId,
    pub basis: EvidenceBasis,
    pub role: EvidenceRole,
    pub causal_basis: CausalBasis,
    pub direction: EvidenceDirection,
    pub diagnosticity: Diagnosticity,
    pub preregistration: PreregistrationStatus,
    pub result_set_status: ResultSetStatus,
    pub replication_kind: ReplicationKind,
    pub dependence_root_id: DependenceRootId,
    pub evidence_artifact_commitment: ArtifactCommitment,
    pub experiment_manifest_commitment: ArtifactCommitment,
    pub result_set_commitment: ArtifactCommitment,
    /// Opaque commitment to immutable claim semantics. AMSAP-005C later owns
    /// canonical encoding, domain separation, and hash-algorithm agility.
    pub claim_payload_commitment: ArtifactCommitment,
    pub verifier: VerifierProfile,
    pub lifecycle: ClaimLifecycle,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QualifiedAxisEvidenceClaim {
    claim: AxisEvidenceClaim,
}

impl QualifiedAxisEvidenceClaim {
    pub const fn claim(&self) -> &AxisEvidenceClaim {
        &self.claim
    }

    pub const fn axis(&self) -> EmpiricalAxis {
        self.claim.axis
    }

    pub fn synthesis_admissibility(&self) -> SynthesisAdmissibility {
        synthesis_admissibility(&self.claim)
    }

    pub fn shares_dependence_root(&self, other: &Self) -> bool {
        self.claim.dependence_root_id == other.claim.dependence_root_id
    }

    pub const fn establishes_scientific_truth(&self) -> bool {
        false
    }

    pub const fn establishes_consciousness(&self) -> bool {
        false
    }

    pub const fn establishes_valence(&self) -> bool {
        false
    }

    pub const fn establishes_responsibility(&self) -> bool {
        false
    }

    pub const fn grants_welfare_protection(&self) -> bool {
        false
    }

    pub const fn establishes_legal_responsibility(&self) -> bool {
        false
    }

    pub const fn grants_liability(&self) -> bool {
        false
    }

    pub const fn grants_legal_standing(&self) -> bool {
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

    /// Replication labeling alone never establishes independence. Independence
    /// remains a relation between verifier/fault-domain profiles.
    pub const fn establishes_replication_independence(&self) -> bool {
        false
    }
}

pub fn qualify_axis_evidence(
    claim: AxisEvidenceClaim,
) -> Result<QualifiedAxisEvidenceClaim, ContractError> {
    validate_basis_role(&claim)?;
    validate_causal_basis(&claim)?;
    Ok(QualifiedAxisEvidenceClaim { claim })
}

/// Detect reuse of one claim identifier for materially different immutable
/// scientific meaning. The payload commitment is an opaque reference here;
/// canonical cryptographic semantics are deferred to AMSAP-005C.
pub fn validate_same_claim_identity(
    left: &QualifiedAxisEvidenceClaim,
    right: &QualifiedAxisEvidenceClaim,
) -> Result<(), ContractError> {
    if left.claim.claim_id != right.claim.claim_id {
        return Ok(());
    }

    let a = &left.claim;
    let b = &right.claim;
    if a.axis != b.axis
        || a.subject_scope != b.subject_scope
        || a.hypothesis_id != b.hypothesis_id
        || a.protocol_id != b.protocol_id
        || a.outcome_spec_id != b.outcome_spec_id
        || a.analysis_plan_id != b.analysis_plan_id
        || a.experiment_id != b.experiment_id
        || a.basis != b.basis
        || a.role != b.role
        || a.causal_basis != b.causal_basis
        || a.direction != b.direction
        || a.diagnosticity != b.diagnosticity
        || a.preregistration != b.preregistration
        || a.result_set_status != b.result_set_status
        || a.replication_kind != b.replication_kind
        || a.dependence_root_id != b.dependence_root_id
        || a.evidence_artifact_commitment != b.evidence_artifact_commitment
        || a.experiment_manifest_commitment != b.experiment_manifest_commitment
        || a.result_set_commitment != b.result_set_commitment
        || a.claim_payload_commitment != b.claim_payload_commitment
        || a.verifier != b.verifier
        || a.lifecycle != b.lifecycle
    {
        return Err(ContractError::ClaimEquivocation);
    }

    Ok(())
}

fn validate_basis_role(claim: &AxisEvidenceClaim) -> Result<(), ContractError> {
    match claim.basis {
        EvidenceBasis::Proxy(_) if claim.role != EvidenceRole::ContextOnly => {
            Err(ContractError::ProxyCannotBeAxisBearing)
        }
        EvidenceBasis::Measurement(
            EvidenceModality::SelfReport
            | EvidenceModality::ExternalReport
            | EvidenceModality::LineageFact,
        ) if claim.role == EvidenceRole::AxisBearing => {
            Err(ContractError::NonQualifyingModalityCannotBeAxisBearing)
        }
        _ => Ok(()),
    }
}

fn validate_causal_basis(claim: &AxisEvidenceClaim) -> Result<(), ContractError> {
    let asserts_intervention = matches!(
        claim.causal_basis,
        CausalBasis::ControlledContrast
            | CausalBasis::InterventionWithShamControl
            | CausalBasis::MechanisticPerturbation
    );

    if !asserts_intervention {
        return Ok(());
    }

    let compatible = matches!(
        claim.basis,
        EvidenceBasis::Measurement(
            EvidenceModality::Mechanistic
                | EvidenceModality::CausalIntervention
                | EvidenceModality::NegativeControl
                | EvidenceModality::AdversarialControl
        )
    );

    if compatible {
        Ok(())
    } else {
        Err(ContractError::CausalBasisModalityMismatch)
    }
}

fn synthesis_admissibility(claim: &AxisEvidenceClaim) -> SynthesisAdmissibility {
    if claim.lifecycle != ClaimLifecycle::Current {
        return SynthesisAdmissibility::NotCurrent;
    }
    if claim.result_set_status != ResultSetStatus::Complete {
        return SynthesisAdmissibility::IncompleteResultSet;
    }
    if matches!(
        claim.preregistration,
        PreregistrationStatus::Exploratory | PreregistrationStatus::PostHoc
    ) {
        return SynthesisAdmissibility::Exploratory;
    }
    if claim.role == EvidenceRole::ContextOnly {
        return SynthesisAdmissibility::ContextOnly;
    }
    if claim.role == EvidenceRole::Corroborative {
        return SynthesisAdmissibility::CorroborativeOnly;
    }
    if claim.diagnosticity != Diagnosticity::Informative {
        return SynthesisAdmissibility::Inconclusive;
    }

    SynthesisAdmissibility::Admissible
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ai_lineage::{InstanceId, LineageId, ModelId, OperationalSubjectRef};
    use mycelix_welfare_evidence::{
        EvaluatorId, EvidenceSourceId, MethodId, OrganizationId, TheoryFamilyId, ToolingId,
    };

    fn bytes(value: u8) -> [u8; 32] {
        [value; 32]
    }

    fn artifact(value: u8) -> ArtifactCommitment {
        ArtifactCommitment::new(bytes(value)).expect("nonzero artifact commitment")
    }

    fn verifier() -> VerifierProfile {
        VerifierProfile {
            organization_id: OrganizationId::new(bytes(31)).expect("organization"),
            evaluator_id: EvaluatorId::new(bytes(32)).expect("evaluator"),
            method_id: MethodId::new(bytes(33)).expect("method"),
            theory_family_id: TheoryFamilyId::new(bytes(34)).expect("theory"),
            tooling_id: ToolingId::new(bytes(35)).expect("tooling"),
            evidence_source_id: EvidenceSourceId::new(bytes(36)).expect("source"),
        }
    }

    fn subject() -> SubjectScope {
        SubjectScope {
            subject: OperationalSubjectRef {
                model_id: ModelId::new(bytes(1)).expect("model"),
                lineage_id: LineageId::new(bytes(2)).expect("lineage"),
                instance_id: InstanceId::new(bytes(3)).expect("instance"),
            },
            developer_organization_id: OrganizationId::new(bytes(4)).expect("developer"),
            configuration_commitment: artifact(5),
            environment_commitment: artifact(6),
            system_policy_commitment: artifact(7),
            scaffold_commitment: Some(artifact(8)),
            memory_root_commitment: Some(artifact(9)),
        }
    }

    fn claim(axis: EmpiricalAxis) -> AxisEvidenceClaim {
        AxisEvidenceClaim {
            claim_id: ClaimId::new(bytes(10)).expect("claim"),
            axis,
            subject_scope: subject(),
            hypothesis_id: HypothesisId::new(bytes(11)).expect("hypothesis"),
            protocol_id: ProtocolId::new(bytes(12)).expect("protocol"),
            outcome_spec_id: OutcomeSpecId::new(bytes(13)).expect("outcome"),
            analysis_plan_id: AnalysisPlanId::new(bytes(14)).expect("analysis"),
            experiment_id: ExperimentId::new(bytes(15)).expect("experiment"),
            basis: EvidenceBasis::Measurement(EvidenceModality::CausalIntervention),
            role: EvidenceRole::AxisBearing,
            causal_basis: CausalBasis::InterventionWithShamControl,
            direction: EvidenceDirection::Supports,
            diagnosticity: Diagnosticity::Informative,
            preregistration: PreregistrationStatus::Preregistered,
            result_set_status: ResultSetStatus::Complete,
            replication_kind: ReplicationKind::NotReplication,
            dependence_root_id: DependenceRootId::new(bytes(16)).expect("dependence root"),
            evidence_artifact_commitment: artifact(17),
            experiment_manifest_commitment: artifact(18),
            result_set_commitment: artifact(19),
            claim_payload_commitment: artifact(20),
            verifier: verifier(),
            lifecycle: ClaimLifecycle::Current,
        }
    }

    #[test]
    fn proxy_cannot_be_relabelled_as_axis_bearing() {
        let mut candidate = claim(EmpiricalAxis::Valence);
        candidate.basis = EvidenceBasis::Proxy(ProxyKind::RewardSignal);
        assert_eq!(
            qualify_axis_evidence(candidate),
            Err(ContractError::ProxyCannotBeAxisBearing)
        );
    }

    #[test]
    fn self_report_alone_cannot_be_axis_bearing() {
        let mut candidate = claim(EmpiricalAxis::Consciousness);
        candidate.basis = EvidenceBasis::Measurement(EvidenceModality::SelfReport);
        candidate.causal_basis = CausalBasis::NoCausalClaim;
        assert_eq!(
            qualify_axis_evidence(candidate),
            Err(ContractError::NonQualifyingModalityCannotBeAxisBearing)
        );
    }

    #[test]
    fn uninformative_null_is_preserved_but_not_synthesis_admissible() {
        let mut candidate = claim(EmpiricalAxis::Valence);
        candidate.direction = EvidenceDirection::Null;
        candidate.diagnosticity = Diagnosticity::Inconclusive;
        let qualified = qualify_axis_evidence(candidate).expect("well-formed null record");
        assert_eq!(
            qualified.synthesis_admissibility(),
            SynthesisAdmissibility::Inconclusive
        );
    }

    #[test]
    fn informative_preregistered_null_can_enter_later_synthesis() {
        let mut candidate = claim(EmpiricalAxis::Valence);
        candidate.direction = EvidenceDirection::Null;
        let qualified = qualify_axis_evidence(candidate).expect("well-formed informative null");
        assert_eq!(
            qualified.synthesis_admissibility(),
            SynthesisAdmissibility::Admissible
        );
    }

    #[test]
    fn incomplete_result_set_is_preserved_but_not_admissible() {
        let mut candidate = claim(EmpiricalAxis::Responsibility);
        candidate.result_set_status = ResultSetStatus::Partial;
        let qualified =
            qualify_axis_evidence(candidate).expect("partial record remains representable");
        assert_eq!(
            qualified.synthesis_admissibility(),
            SynthesisAdmissibility::IncompleteResultSet
        );
    }

    #[test]
    fn same_artifact_across_axes_remains_two_claims_with_one_dependence_root() {
        let c = qualify_axis_evidence(claim(EmpiricalAxis::Consciousness)).expect("C claim");
        let mut v_claim = claim(EmpiricalAxis::Valence);
        v_claim.claim_id = ClaimId::new(bytes(21)).expect("second claim");
        let v = qualify_axis_evidence(v_claim).expect("V claim");
        assert_ne!(c.axis(), v.axis());
        assert!(c.shares_dependence_root(&v));
        assert!(!c.establishes_replication_independence());
    }

    #[test]
    fn same_claim_id_cannot_change_axis_payload_or_result() {
        let c = qualify_axis_evidence(claim(EmpiricalAxis::Consciousness)).expect("C claim");
        let v = qualify_axis_evidence(claim(EmpiricalAxis::Valence)).expect("V claim");
        assert_eq!(
            validate_same_claim_identity(&c, &v),
            Err(ContractError::ClaimEquivocation)
        );

        let mut changed = claim(EmpiricalAxis::Consciousness);
        changed.claim_payload_commitment = artifact(99);
        let changed = qualify_axis_evidence(changed).expect("changed payload");
        assert_eq!(
            validate_same_claim_identity(&c, &changed),
            Err(ContractError::ClaimEquivocation)
        );

        let mut changed_result = claim(EmpiricalAxis::Consciousness);
        changed_result.direction = EvidenceDirection::Challenges;
        let changed_result = qualify_axis_evidence(changed_result).expect("changed result");
        assert_eq!(
            validate_same_claim_identity(&c, &changed_result),
            Err(ContractError::ClaimEquivocation)
        );
    }

    #[test]
    fn replication_provenance_does_not_manufacture_independence() {
        let mut replication = claim(EmpiricalAxis::Consciousness);
        replication.replication_kind = ReplicationKind::NewRunIndependentImplementation;
        let qualified = qualify_axis_evidence(replication).expect("well-formed replication claim");
        assert!(!qualified.establishes_replication_independence());
    }

    #[test]
    fn causal_claim_requires_compatible_modality() {
        let mut bad = claim(EmpiricalAxis::Responsibility);
        bad.basis = EvidenceBasis::Measurement(EvidenceModality::Behavioral);
        bad.causal_basis = CausalBasis::MechanisticPerturbation;
        assert_eq!(
            qualify_axis_evidence(bad),
            Err(ContractError::CausalBasisModalityMismatch)
        );
    }

    #[test]
    fn exploratory_evidence_is_preserved_without_admissibility() {
        let mut candidate = claim(EmpiricalAxis::Consciousness);
        candidate.preregistration = PreregistrationStatus::Exploratory;
        let qualified = qualify_axis_evidence(candidate).expect("exploratory claim retained");
        assert_eq!(
            qualified.synthesis_admissibility(),
            SynthesisAdmissibility::Exploratory
        );
    }

    #[test]
    fn supportive_but_undiagnostic_evidence_is_inconclusive() {
        let mut candidate = claim(EmpiricalAxis::Responsibility);
        candidate.diagnosticity = Diagnosticity::Unassessed;
        let qualified = qualify_axis_evidence(candidate).expect("record retained");
        assert_eq!(
            qualified.synthesis_admissibility(),
            SynthesisAdmissibility::Inconclusive
        );
    }

    #[test]
    fn qualified_claim_grants_no_truth_or_authority() {
        let qualified = qualify_axis_evidence(claim(EmpiricalAxis::Responsibility))
            .expect("well-formed responsibility claim");
        assert!(!qualified.establishes_scientific_truth());
        assert!(!qualified.establishes_consciousness());
        assert!(!qualified.establishes_valence());
        assert!(!qualified.establishes_responsibility());
        assert!(!qualified.grants_welfare_protection());
        assert!(!qualified.establishes_legal_responsibility());
        assert!(!qualified.grants_liability());
        assert!(!qualified.grants_legal_standing());
        assert!(!qualified.grants_currentness());
        assert!(!qualified.grants_deployment_authority());
        assert!(!qualified.grants_governance_authority());
        assert!(!qualified.grants_external_effect_authority());
    }
}
