// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Authority-neutral FHE semantics layered over Mycelix PEC.
//!
//! This crate implements no homomorphic encryption. It describes exact numeric,
//! key, operation, bootstrapping, target, and integrity requirements so an
//! FHE-shaped API cannot be mistaken for a qualified privacy capability.

use privacy_computation_core::{
    evaluate_capability, CompositionFailure, CompositionDisposition, PrimitiveCapability,
    PrivacyObjective, PrivacyPrimitive, PrivacyRequirement, SemanticAuthority,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FheSchemeFamily {
    Bfv,
    Bgv,
    Ckks,
    BooleanOrIntegerGate,
    BackendSpecific,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FheNumericModel {
    ExactInteger,
    ApproximateReal,
    Boolean,
    FixedWidthInteger { bits: u16 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FheKeyModel {
    SingleKey,
    Threshold { threshold: u16, participants: u16 },
    MultiKey { parties: u16 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FheOperationClass {
    Addition,
    Multiplication,
    Comparison,
    BooleanGate,
    Rotation,
    Lookup,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BootstrappingRequirement {
    Unspecified,
    Required,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BootstrappingDeclaration {
    Unsupported,
    DeclaredSupported,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ExecutionTarget {
    Cpu,
    Gpu,
    Wasm,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EvaluationIntegrityRequirement {
    Unspecified,
    Required,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EvaluationIntegrityDeclaration {
    Unspecified,
    NotProvided,
    DeclaredProvided,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FheRequirement {
    pub pec: PrivacyRequirement,
    pub scheme_family: FheSchemeFamily,
    pub numeric_model: FheNumericModel,
    pub key_model: FheKeyModel,
    pub required_operations: Vec<FheOperationClass>,
    pub bootstrapping: BootstrappingRequirement,
    pub execution_target: ExecutionTarget,
    pub evaluation_integrity: EvaluationIntegrityRequirement,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FheCapability {
    pub pec: PrimitiveCapability,
    pub scheme_family: FheSchemeFamily,
    pub numeric_model: FheNumericModel,
    pub key_model: FheKeyModel,
    pub operations: Vec<FheOperationClass>,
    pub bootstrapping: BootstrappingDeclaration,
    pub execution_targets: Vec<ExecutionTarget>,
    /// Exact backend-defined parameter/profile identity. Presence does not prove
    /// security strength; qualification belongs to the evidence layer.
    pub parameter_profile: String,
    pub evaluation_integrity: EvaluationIntegrityDeclaration,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FheFailure {
    WrongPrimitive,
    WrongObjective,
    PecIncompatible(CompositionFailure),
    SchemeMismatch,
    NumericModelMismatch,
    KeyModelMismatch,
    UnsupportedOperation(FheOperationClass),
    BootstrappingUnavailable,
    ExecutionTargetUnavailable,
    MissingParameterProfile,
    EvaluationIntegrityUnavailable,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FheDisposition {
    Compatible,
    Incompatible(FheFailure),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FheEvaluation {
    pub disposition: FheDisposition,
    pub authority: SemanticAuthority,
}

impl FheEvaluation {
    fn compatible() -> Self {
        Self {
            disposition: FheDisposition::Compatible,
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    fn incompatible(reason: FheFailure) -> Self {
        Self {
            disposition: FheDisposition::Incompatible(reason),
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    pub const fn ciphertext_confidentiality_established(&self) -> bool {
        false
    }

    pub const fn parameter_security_established(&self) -> bool {
        false
    }

    pub const fn numeric_correctness_established(&self) -> bool {
        false
    }

    pub const fn threshold_security_established(&self) -> bool {
        false
    }

    pub const fn evaluation_integrity_established(&self) -> bool {
        false
    }

    pub const fn production_admission_granted(&self) -> bool {
        false
    }
}

pub fn evaluate_fhe(
    requirement: &FheRequirement,
    capability: &FheCapability,
) -> FheEvaluation {
    if capability.pec.backend.primitive != PrivacyPrimitive::HomomorphicEncryption {
        return FheEvaluation::incompatible(FheFailure::WrongPrimitive);
    }

    if requirement.pec.objective != PrivacyObjective::ComputeOnCiphertext {
        return FheEvaluation::incompatible(FheFailure::WrongObjective);
    }

    let pec = evaluate_capability(&requirement.pec, &capability.pec);
    if let CompositionDisposition::Incompatible(reason) = pec.disposition {
        return FheEvaluation::incompatible(FheFailure::PecIncompatible(reason));
    }

    if capability.scheme_family != requirement.scheme_family {
        return FheEvaluation::incompatible(FheFailure::SchemeMismatch);
    }

    if capability.numeric_model != requirement.numeric_model {
        return FheEvaluation::incompatible(FheFailure::NumericModelMismatch);
    }

    if capability.key_model != requirement.key_model {
        return FheEvaluation::incompatible(FheFailure::KeyModelMismatch);
    }

    for operation in &requirement.required_operations {
        if !capability.operations.contains(operation) {
            return FheEvaluation::incompatible(FheFailure::UnsupportedOperation(*operation));
        }
    }

    if matches!(requirement.bootstrapping, BootstrappingRequirement::Required)
        && !matches!(
            capability.bootstrapping,
            BootstrappingDeclaration::DeclaredSupported
        )
    {
        return FheEvaluation::incompatible(FheFailure::BootstrappingUnavailable);
    }

    if !capability
        .execution_targets
        .contains(&requirement.execution_target)
    {
        return FheEvaluation::incompatible(FheFailure::ExecutionTargetUnavailable);
    }

    if capability.parameter_profile.trim().is_empty() {
        return FheEvaluation::incompatible(FheFailure::MissingParameterProfile);
    }

    if matches!(
        requirement.evaluation_integrity,
        EvaluationIntegrityRequirement::Required
    ) && !matches!(
        capability.evaluation_integrity,
        EvaluationIntegrityDeclaration::DeclaredProvided
    ) {
        return FheEvaluation::incompatible(FheFailure::EvaluationIntegrityUnavailable);
    }

    FheEvaluation::compatible()
}

#[cfg(test)]
mod tests {
    use super::*;
    use privacy_computation_core::{
        AdversaryModel, BackendIdentity, InteractionModel, LeakageProfile, LeakageRequirements,
        ParticipantModel, QualificationState,
    };

    fn requirement() -> FheRequirement {
        FheRequirement {
            pec: PrivacyRequirement {
                objective: PrivacyObjective::ComputeOnCiphertext,
                participant_model: ParticipantModel::SingleParty,
                adversary_model: AdversaryModel::HonestButCurious,
                interaction_model: InteractionModel::NonInteractive,
                leakage: LeakageRequirements::default(),
                required_qualification: Some(QualificationState::Experimental),
            },
            scheme_family: FheSchemeFamily::Bfv,
            numeric_model: FheNumericModel::ExactInteger,
            key_model: FheKeyModel::SingleKey,
            required_operations: vec![FheOperationClass::Addition],
            bootstrapping: BootstrappingRequirement::Unspecified,
            execution_target: ExecutionTarget::Cpu,
            evaluation_integrity: EvaluationIntegrityRequirement::Unspecified,
        }
    }

    fn capability() -> FheCapability {
        FheCapability {
            pec: PrimitiveCapability {
                backend: BackendIdentity {
                    primitive: PrivacyPrimitive::HomomorphicEncryption,
                    backend: "synthetic-fhe".into(),
                    version: "0.0.1".into(),
                    profile: "bfv-demo".into(),
                },
                supported_objectives: vec![PrivacyObjective::ComputeOnCiphertext],
                participant_model: ParticipantModel::SingleParty,
                adversary_model: AdversaryModel::HonestButCurious,
                interaction_model: InteractionModel::NonInteractive,
                leakage: LeakageProfile::default(),
                qualification: QualificationState::Experimental,
            },
            scheme_family: FheSchemeFamily::Bfv,
            numeric_model: FheNumericModel::ExactInteger,
            key_model: FheKeyModel::SingleKey,
            operations: vec![FheOperationClass::Addition, FheOperationClass::Multiplication],
            bootstrapping: BootstrappingDeclaration::Unsupported,
            execution_targets: vec![ExecutionTarget::Cpu],
            parameter_profile: "synthetic-no-security-claim".into(),
            evaluation_integrity: EvaluationIntegrityDeclaration::NotProvided,
        }
    }

    #[test]
    fn compatible_remains_structural_only() {
        let result = evaluate_fhe(&requirement(), &capability());
        assert_eq!(result.disposition, FheDisposition::Compatible);
        assert_eq!(result.authority, SemanticAuthority::StructuralOnly);
        assert!(!result.ciphertext_confidentiality_established());
        assert!(!result.parameter_security_established());
        assert!(!result.numeric_correctness_established());
        assert!(!result.threshold_security_established());
        assert!(!result.evaluation_integrity_established());
        assert!(!result.production_admission_granted());
    }

    #[test]
    fn approximate_real_does_not_substitute_for_exact_integer() {
        let mut cap = capability();
        cap.scheme_family = FheSchemeFamily::Ckks;
        cap.numeric_model = FheNumericModel::ApproximateReal;
        assert!(matches!(
            evaluate_fhe(&requirement(), &cap).disposition,
            FheDisposition::Incompatible(FheFailure::SchemeMismatch | FheFailure::NumericModelMismatch)
        ));
    }

    #[test]
    fn single_key_does_not_substitute_for_threshold_fhe() {
        let mut req = requirement();
        req.key_model = FheKeyModel::Threshold {
            threshold: 2,
            participants: 3,
        };
        assert_eq!(
            evaluate_fhe(&req, &capability()).disposition,
            FheDisposition::Incompatible(FheFailure::KeyModelMismatch)
        );
    }

    #[test]
    fn bootstrapping_requirement_fails_closed() {
        let mut req = requirement();
        req.bootstrapping = BootstrappingRequirement::Required;
        assert_eq!(
            evaluate_fhe(&req, &capability()).disposition,
            FheDisposition::Incompatible(FheFailure::BootstrappingUnavailable)
        );
    }

    #[test]
    fn evaluation_integrity_is_not_implied_by_fhe() {
        let mut req = requirement();
        req.evaluation_integrity = EvaluationIntegrityRequirement::Required;
        assert_eq!(
            evaluate_fhe(&req, &capability()).disposition,
            FheDisposition::Incompatible(FheFailure::EvaluationIntegrityUnavailable)
        );
    }

    #[test]
    fn missing_parameter_identity_fails_closed() {
        let mut cap = capability();
        cap.parameter_profile.clear();
        assert_eq!(
            evaluate_fhe(&requirement(), &cap).disposition,
            FheDisposition::Incompatible(FheFailure::MissingParameterProfile)
        );
    }

    #[test]
    fn backend_name_does_not_mint_security() {
        let mut cap = capability();
        cap.pec.backend.backend = "production-super-secure-fhe".into();
        let result = evaluate_fhe(&requirement(), &cap);
        assert_eq!(result.disposition, FheDisposition::Compatible);
        assert!(!result.ciphertext_confidentiality_established());
        assert!(!result.parameter_security_established());
    }

    #[test]
    fn core_wire_names_are_stable() {
        assert_eq!(
            serde_json::to_string(&FheNumericModel::ApproximateReal).unwrap(),
            "\"ApproximateReal\""
        );
        assert_eq!(
            serde_json::to_string(&FheKeyModel::SingleKey).unwrap(),
            "\"SingleKey\""
        );
    }
}
