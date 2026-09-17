// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![no_std]

use mycelix_evidence_contract::{Diagnosticity, QualifiedAxisEvidenceClaim};
use mycelix_welfare_evidence::{ArtifactCommitment, ClaimId};

pub const PROTOCOL_VERSION: &str = "mycelix-amsap-evidence-assurance-v0.1";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssuranceError {
    ClaimBindingMismatch,
    MissingDiagnosticityReceipt,
    UnexpectedDiagnosticityReceipt,
    DiagnosticityClaimMismatch,
    MissingCounterevidenceSearch,
    CounterevidenceSearchMismatch,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum EvidenceMagnitude {
    Unassessed,
    Weak,
    Moderate,
    Strong,
    Exceptional,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum CounterevidenceStrength {
    Unassessed,
    NoneDetected,
    Weak,
    Moderate,
    Strong,
    Exceptional,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Contestation {
    Unassessed,
    None,
    Low,
    Moderate,
    High,
    Fundamental,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SensitivityAssessment {
    Unassessed,
    Insufficient,
    QualifiedForClaim,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ControlAssessment {
    Unassessed,
    ExpectedBehaviorNotObserved,
    ExpectedBehaviorObserved,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EarnedDiagnosticity {
    Unassessed,
    Inconclusive,
    Informative,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DiagnosticityReceipt {
    pub claim_id: ClaimId,
    pub claim_payload_commitment: ArtifactCommitment,
    pub predicted_observation_commitment: ArtifactCommitment,
    pub sensitivity_bound_commitment: ArtifactCommitment,
    pub positive_control_commitment: ArtifactCommitment,
    pub sham_or_negative_control_commitment: ArtifactCommitment,
    pub observation_horizon_commitment: ArtifactCommitment,
    pub stopping_rule_commitment: ArtifactCommitment,
    pub sensitivity: SensitivityAssessment,
    pub positive_control: ControlAssessment,
    pub sham_or_negative_control: ControlAssessment,
}

impl DiagnosticityReceipt {
    pub const fn earned_diagnosticity(&self) -> EarnedDiagnosticity {
        if matches!(self.sensitivity, SensitivityAssessment::Unassessed)
            && matches!(self.positive_control, ControlAssessment::Unassessed)
            && matches!(self.sham_or_negative_control, ControlAssessment::Unassessed)
        {
            return EarnedDiagnosticity::Unassessed;
        }

        if matches!(self.sensitivity, SensitivityAssessment::QualifiedForClaim)
            && matches!(
                self.positive_control,
                ControlAssessment::ExpectedBehaviorObserved
            )
            && matches!(
                self.sham_or_negative_control,
                ControlAssessment::ExpectedBehaviorObserved
            )
        {
            return EarnedDiagnosticity::Informative;
        }

        EarnedDiagnosticity::Inconclusive
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CounterevidenceSearchCoverage {
    Unassessed,
    NarrowButInformative,
    AdequateForClaim,
    Insufficient,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CounterevidenceSearchOutcome {
    Unassessed,
    NoQualifiedCounterevidenceFound,
    QualifiedCounterevidenceFound,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CounterevidenceSearchReceipt {
    pub claim_id: ClaimId,
    pub claim_payload_commitment: ArtifactCommitment,
    pub search_protocol_commitment: ArtifactCommitment,
    pub search_space_commitment: ArtifactCommitment,
    pub exclusion_rule_commitment: ArtifactCommitment,
    pub coverage: CounterevidenceSearchCoverage,
    pub outcome: CounterevidenceSearchOutcome,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EvidenceAssuranceProfile {
    pub evidence_magnitude: EvidenceMagnitude,
    pub counterevidence_strength: CounterevidenceStrength,
    pub contestation: Contestation,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QualifiedEvidenceAssurance {
    claim: QualifiedAxisEvidenceClaim,
    profile: EvidenceAssuranceProfile,
    earned_diagnosticity: EarnedDiagnosticity,
}

impl QualifiedEvidenceAssurance {
    pub const fn claim(&self) -> &QualifiedAxisEvidenceClaim {
        &self.claim
    }

    pub const fn profile(&self) -> EvidenceAssuranceProfile {
        self.profile
    }

    pub const fn earned_diagnosticity(&self) -> EarnedDiagnosticity {
        self.earned_diagnosticity
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

pub fn qualify_evidence_assurance(
    claim: QualifiedAxisEvidenceClaim,
    profile: EvidenceAssuranceProfile,
    diagnosticity_receipt: Option<DiagnosticityReceipt>,
    counterevidence_search: Option<CounterevidenceSearchReceipt>,
) -> Result<QualifiedEvidenceAssurance, AssuranceError> {
    let earned_diagnosticity = validate_diagnosticity(&claim, diagnosticity_receipt)?;
    validate_counterevidence(
        &claim,
        profile.counterevidence_strength,
        counterevidence_search,
    )?;

    Ok(QualifiedEvidenceAssurance {
        claim,
        profile,
        earned_diagnosticity,
    })
}

fn validate_claim_binding(
    claim: &QualifiedAxisEvidenceClaim,
    claim_id: ClaimId,
    claim_payload_commitment: ArtifactCommitment,
) -> Result<(), AssuranceError> {
    let inner = claim.claim();
    if inner.claim_id != claim_id || inner.claim_payload_commitment != claim_payload_commitment {
        return Err(AssuranceError::ClaimBindingMismatch);
    }
    Ok(())
}

fn validate_diagnosticity(
    claim: &QualifiedAxisEvidenceClaim,
    receipt: Option<DiagnosticityReceipt>,
) -> Result<EarnedDiagnosticity, AssuranceError> {
    match (claim.claim().diagnosticity, receipt) {
        (Diagnosticity::Unassessed, None) => Ok(EarnedDiagnosticity::Unassessed),
        (Diagnosticity::Unassessed, Some(_)) => Err(AssuranceError::UnexpectedDiagnosticityReceipt),
        (Diagnosticity::Inconclusive | Diagnosticity::Informative, None) => {
            Err(AssuranceError::MissingDiagnosticityReceipt)
        }
        (expected, Some(receipt)) => {
            validate_claim_binding(claim, receipt.claim_id, receipt.claim_payload_commitment)?;
            let earned = receipt.earned_diagnosticity();
            let matches_claim = matches!(
                (expected, earned),
                (
                    Diagnosticity::Inconclusive,
                    EarnedDiagnosticity::Inconclusive
                ) | (Diagnosticity::Informative, EarnedDiagnosticity::Informative)
            );
            if !matches_claim {
                return Err(AssuranceError::DiagnosticityClaimMismatch);
            }
            Ok(earned)
        }
    }
}

fn validate_counterevidence(
    claim: &QualifiedAxisEvidenceClaim,
    strength: CounterevidenceStrength,
    receipt: Option<CounterevidenceSearchReceipt>,
) -> Result<(), AssuranceError> {
    match strength {
        CounterevidenceStrength::Unassessed => Ok(()),
        CounterevidenceStrength::NoneDetected => {
            let receipt = receipt.ok_or(AssuranceError::MissingCounterevidenceSearch)?;
            validate_claim_binding(claim, receipt.claim_id, receipt.claim_payload_commitment)?;
            let adequate = matches!(
                receipt.coverage,
                CounterevidenceSearchCoverage::AdequateForClaim
            );
            let none_found = matches!(
                receipt.outcome,
                CounterevidenceSearchOutcome::NoQualifiedCounterevidenceFound
            );
            if adequate && none_found {
                Ok(())
            } else {
                Err(AssuranceError::CounterevidenceSearchMismatch)
            }
        }
        CounterevidenceStrength::Weak
        | CounterevidenceStrength::Moderate
        | CounterevidenceStrength::Strong
        | CounterevidenceStrength::Exceptional => {
            let receipt = receipt.ok_or(AssuranceError::MissingCounterevidenceSearch)?;
            validate_claim_binding(claim, receipt.claim_id, receipt.claim_payload_commitment)?;
            if matches!(
                receipt.outcome,
                CounterevidenceSearchOutcome::QualifiedCounterevidenceFound
            ) {
                Ok(())
            } else {
                Err(AssuranceError::CounterevidenceSearchMismatch)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ai_lineage::{InstanceId, LineageId, ModelId, OperationalSubjectRef};
    use mycelix_evidence_contract::{
        AnalysisPlanId, AxisEvidenceClaim, CausalBasis, ClaimLifecycle, DependenceRootId,
        Diagnosticity, EmpiricalAxis, EvidenceBasis, EvidenceModality, EvidenceRole, ExperimentId,
        HypothesisId, PreregistrationStatus, ProtocolId, ReplicationKind, ResultSetStatus,
        qualify_axis_evidence,
    };
    use mycelix_welfare_evidence::{
        EvaluatorId, EvidenceDirection, EvidenceSourceId, MethodId, OrganizationId, SubjectScope,
        TheoryFamilyId, ToolingId, VerifierProfile,
    };

    fn bytes(value: u8) -> [u8; 32] {
        [value; 32]
    }

    fn artifact(value: u8) -> ArtifactCommitment {
        ArtifactCommitment::new(bytes(value)).expect("artifact")
    }

    fn claim(diagnosticity: Diagnosticity) -> QualifiedAxisEvidenceClaim {
        let subject = SubjectScope {
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
        };
        let verifier = VerifierProfile {
            organization_id: OrganizationId::new(bytes(31)).expect("organization"),
            evaluator_id: EvaluatorId::new(bytes(32)).expect("evaluator"),
            method_id: MethodId::new(bytes(33)).expect("method"),
            theory_family_id: TheoryFamilyId::new(bytes(34)).expect("theory"),
            tooling_id: ToolingId::new(bytes(35)).expect("tooling"),
            evidence_source_id: EvidenceSourceId::new(bytes(36)).expect("source"),
        };
        qualify_axis_evidence(AxisEvidenceClaim {
            claim_id: ClaimId::new(bytes(10)).expect("claim"),
            axis: EmpiricalAxis::Responsibility,
            subject_scope: subject,
            hypothesis_id: HypothesisId::new(bytes(11)).expect("hypothesis"),
            protocol_id: ProtocolId::new(bytes(12)).expect("protocol"),
            outcome_spec_id: mycelix_evidence_contract::OutcomeSpecId::new(bytes(13))
                .expect("outcome"),
            analysis_plan_id: AnalysisPlanId::new(bytes(14)).expect("analysis"),
            experiment_id: ExperimentId::new(bytes(15)).expect("experiment"),
            basis: EvidenceBasis::Measurement(EvidenceModality::CausalIntervention),
            role: EvidenceRole::AxisBearing,
            causal_basis: CausalBasis::InterventionWithShamControl,
            direction: EvidenceDirection::Supports,
            diagnosticity,
            preregistration: PreregistrationStatus::Preregistered,
            result_set_status: ResultSetStatus::Complete,
            replication_kind: ReplicationKind::NotReplication,
            dependence_root_id: DependenceRootId::new(bytes(16)).expect("dependence root"),
            evidence_artifact_commitment: artifact(17),
            experiment_manifest_commitment: artifact(18),
            result_set_commitment: artifact(19),
            claim_payload_commitment: artifact(20),
            verifier,
            lifecycle: ClaimLifecycle::Current,
        })
        .expect("qualified claim")
    }

    fn diagnosticity_receipt() -> DiagnosticityReceipt {
        DiagnosticityReceipt {
            claim_id: ClaimId::new(bytes(10)).expect("claim"),
            claim_payload_commitment: artifact(20),
            predicted_observation_commitment: artifact(40),
            sensitivity_bound_commitment: artifact(41),
            positive_control_commitment: artifact(42),
            sham_or_negative_control_commitment: artifact(43),
            observation_horizon_commitment: artifact(44),
            stopping_rule_commitment: artifact(45),
            sensitivity: SensitivityAssessment::QualifiedForClaim,
            positive_control: ControlAssessment::ExpectedBehaviorObserved,
            sham_or_negative_control: ControlAssessment::ExpectedBehaviorObserved,
        }
    }

    fn search_receipt(
        outcome: CounterevidenceSearchOutcome,
        coverage: CounterevidenceSearchCoverage,
    ) -> CounterevidenceSearchReceipt {
        CounterevidenceSearchReceipt {
            claim_id: ClaimId::new(bytes(10)).expect("claim"),
            claim_payload_commitment: artifact(20),
            search_protocol_commitment: artifact(50),
            search_space_commitment: artifact(51),
            exclusion_rule_commitment: artifact(52),
            coverage,
            outcome,
        }
    }

    #[test]
    fn strong_evidence_and_high_contestation_can_coexist() {
        let qualified = qualify_evidence_assurance(
            claim(Diagnosticity::Informative),
            EvidenceAssuranceProfile {
                evidence_magnitude: EvidenceMagnitude::Strong,
                counterevidence_strength: CounterevidenceStrength::NoneDetected,
                contestation: Contestation::High,
            },
            Some(diagnosticity_receipt()),
            Some(search_receipt(
                CounterevidenceSearchOutcome::NoQualifiedCounterevidenceFound,
                CounterevidenceSearchCoverage::AdequateForClaim,
            )),
        )
        .expect("qualified assurance");
        assert_eq!(
            qualified.profile().evidence_magnitude,
            EvidenceMagnitude::Strong
        );
        assert_eq!(qualified.profile().contestation, Contestation::High);
    }

    #[test]
    fn informative_diagnosticity_requires_a_receipt() {
        assert_eq!(
            qualify_evidence_assurance(
                claim(Diagnosticity::Informative),
                EvidenceAssuranceProfile {
                    evidence_magnitude: EvidenceMagnitude::Moderate,
                    counterevidence_strength: CounterevidenceStrength::Unassessed,
                    contestation: Contestation::Unassessed,
                },
                None,
                None,
            ),
            Err(AssuranceError::MissingDiagnosticityReceipt)
        );
    }

    #[test]
    fn failed_positive_control_cannot_earn_informative() {
        let mut receipt = diagnosticity_receipt();
        receipt.positive_control = ControlAssessment::ExpectedBehaviorNotObserved;
        assert_eq!(
            qualify_evidence_assurance(
                claim(Diagnosticity::Informative),
                EvidenceAssuranceProfile {
                    evidence_magnitude: EvidenceMagnitude::Moderate,
                    counterevidence_strength: CounterevidenceStrength::Unassessed,
                    contestation: Contestation::Low,
                },
                Some(receipt),
                None,
            ),
            Err(AssuranceError::DiagnosticityClaimMismatch)
        );
    }

    #[test]
    fn none_detected_requires_adequate_search() {
        assert_eq!(
            qualify_evidence_assurance(
                claim(Diagnosticity::Informative),
                EvidenceAssuranceProfile {
                    evidence_magnitude: EvidenceMagnitude::Strong,
                    counterevidence_strength: CounterevidenceStrength::NoneDetected,
                    contestation: Contestation::None,
                },
                Some(diagnosticity_receipt()),
                Some(search_receipt(
                    CounterevidenceSearchOutcome::NoQualifiedCounterevidenceFound,
                    CounterevidenceSearchCoverage::NarrowButInformative,
                )),
            ),
            Err(AssuranceError::CounterevidenceSearchMismatch)
        );
    }

    #[test]
    fn observed_counterevidence_cannot_be_none_detected() {
        assert_eq!(
            qualify_evidence_assurance(
                claim(Diagnosticity::Informative),
                EvidenceAssuranceProfile {
                    evidence_magnitude: EvidenceMagnitude::Strong,
                    counterevidence_strength: CounterevidenceStrength::NoneDetected,
                    contestation: Contestation::Low,
                },
                Some(diagnosticity_receipt()),
                Some(search_receipt(
                    CounterevidenceSearchOutcome::QualifiedCounterevidenceFound,
                    CounterevidenceSearchCoverage::AdequateForClaim,
                )),
            ),
            Err(AssuranceError::CounterevidenceSearchMismatch)
        );
    }

    #[test]
    fn strong_counterevidence_does_not_force_high_contestation() {
        let qualified = qualify_evidence_assurance(
            claim(Diagnosticity::Informative),
            EvidenceAssuranceProfile {
                evidence_magnitude: EvidenceMagnitude::Strong,
                counterevidence_strength: CounterevidenceStrength::Strong,
                contestation: Contestation::Low,
            },
            Some(diagnosticity_receipt()),
            Some(search_receipt(
                CounterevidenceSearchOutcome::QualifiedCounterevidenceFound,
                CounterevidenceSearchCoverage::AdequateForClaim,
            )),
        )
        .expect("qualified assurance");
        assert_eq!(
            qualified.profile().counterevidence_strength,
            CounterevidenceStrength::Strong
        );
        assert_eq!(qualified.profile().contestation, Contestation::Low);
    }

    #[test]
    fn assurance_grants_no_authority_or_status() {
        let qualified = qualify_evidence_assurance(
            claim(Diagnosticity::Informative),
            EvidenceAssuranceProfile {
                evidence_magnitude: EvidenceMagnitude::Exceptional,
                counterevidence_strength: CounterevidenceStrength::NoneDetected,
                contestation: Contestation::None,
            },
            Some(diagnosticity_receipt()),
            Some(search_receipt(
                CounterevidenceSearchOutcome::NoQualifiedCounterevidenceFound,
                CounterevidenceSearchCoverage::AdequateForClaim,
            )),
        )
        .expect("qualified assurance");
        assert!(!qualified.establishes_scientific_truth());
        assert!(!qualified.establishes_consciousness());
        assert!(!qualified.establishes_valence());
        assert!(!qualified.establishes_responsibility());
        assert!(!qualified.grants_welfare_protection());
        assert!(!qualified.establishes_legal_responsibility());
        assert!(!qualified.grants_currentness());
        assert!(!qualified.grants_deployment_authority());
        assert!(!qualified.grants_governance_authority());
        assert!(!qualified.grants_external_effect_authority());
    }
}
