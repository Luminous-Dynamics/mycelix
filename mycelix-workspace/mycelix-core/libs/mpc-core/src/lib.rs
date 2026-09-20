// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Authority-neutral general MPC semantics layered over Mycelix PEC.
//!
//! This crate implements no MPC protocol. It prevents secret sharing, secure
//! aggregation, threshold cryptography, or MPC-shaped APIs from being treated
//! as interchangeable security profiles.

use privacy_computation_core::{
    evaluate_capability, CompositionFailure, CompositionDisposition, PrimitiveCapability,
    PrivacyObjective, PrivacyPrimitive, PrivacyRequirement, SemanticAuthority,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MpcComputationClass {
    SecureAggregation,
    ArithmeticCircuit,
    BooleanCircuit,
    QualifiedCustomFunction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CorruptionBehavior {
    SemiHonest,
    Malicious,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CorruptionAdaptivity {
    Static,
    Adaptive,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct MpcCorruptionModel {
    pub behavior: CorruptionBehavior,
    pub adaptivity: CorruptionAdaptivity,
    pub max_corrupt_parties: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FairnessRequirement {
    Unspecified,
    Required,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FairnessDeclaration {
    Unspecified,
    NotProvided,
    DeclaredProvided,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum OutputDeliveryRequirement {
    AbortAllowed,
    GuaranteedOutputRequired,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum OutputDeliveryDeclaration {
    AbortMayPreventOutput,
    DeclaredGuaranteedOutput,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MpcSetupModel {
    None,
    PublicParameters,
    TrustedDealer,
    CorrelatedRandomness,
    OfflineMpcPreprocessing,
    ProtocolSpecific,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DropoutModel {
    NoDropout,
    ToleratesUpTo { parties: u16 },
    ProtocolSpecific,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MpcRequirement {
    pub pec: PrivacyRequirement,
    pub computation_class: MpcComputationClass,
    pub corruption_model: MpcCorruptionModel,
    pub fairness: FairnessRequirement,
    pub output_delivery: OutputDeliveryRequirement,
    pub setup_model: MpcSetupModel,
    pub dropout_model: DropoutModel,
    /// Exact circuit/program/function identity. A human-readable function name
    /// is insufficient for an authority-bearing profile.
    pub computation_id: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MpcCapability {
    pub pec: PrimitiveCapability,
    pub computation_classes: Vec<MpcComputationClass>,
    pub corruption_model: MpcCorruptionModel,
    pub fairness: FairnessDeclaration,
    pub output_delivery: OutputDeliveryDeclaration,
    pub setup_models: Vec<MpcSetupModel>,
    pub dropout_model: DropoutModel,
    pub qualified_computation_ids: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MpcFailure {
    WrongPrimitive,
    WrongObjective,
    PecIncompatible(CompositionFailure),
    UnsupportedComputationClass,
    CorruptionBehaviorMismatch,
    CorruptionAdaptivityMismatch,
    CorruptionThresholdInsufficient,
    FairnessUnavailable,
    GuaranteedOutputUnavailable,
    SetupModelUnsupported,
    DropoutModelMismatch,
    MissingComputationIdentity,
    ComputationIdentityUnsupported,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MpcDisposition {
    Compatible,
    Incompatible(MpcFailure),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MpcEvaluation {
    pub disposition: MpcDisposition,
    pub authority: SemanticAuthority,
}

impl MpcEvaluation {
    fn compatible() -> Self {
        Self {
            disposition: MpcDisposition::Compatible,
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    fn incompatible(reason: MpcFailure) -> Self {
        Self {
            disposition: MpcDisposition::Incompatible(reason),
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    pub const fn input_privacy_established(&self) -> bool {
        false
    }

    pub const fn malicious_security_established(&self) -> bool {
        false
    }

    pub const fn fairness_established(&self) -> bool {
        false
    }

    pub const fn guaranteed_output_established(&self) -> bool {
        false
    }

    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

pub fn evaluate_mpc(
    requirement: &MpcRequirement,
    capability: &MpcCapability,
) -> MpcEvaluation {
    if capability.pec.backend.primitive != PrivacyPrimitive::MultiPartyComputation {
        return MpcEvaluation::incompatible(MpcFailure::WrongPrimitive);
    }

    if requirement.pec.objective != PrivacyObjective::JointInputConfidentiality {
        return MpcEvaluation::incompatible(MpcFailure::WrongObjective);
    }

    let pec = evaluate_capability(&requirement.pec, &capability.pec);
    if let CompositionDisposition::Incompatible(reason) = pec.disposition {
        return MpcEvaluation::incompatible(MpcFailure::PecIncompatible(reason));
    }

    if !capability
        .computation_classes
        .contains(&requirement.computation_class)
    {
        return MpcEvaluation::incompatible(MpcFailure::UnsupportedComputationClass);
    }

    if capability.corruption_model.behavior != requirement.corruption_model.behavior {
        return MpcEvaluation::incompatible(MpcFailure::CorruptionBehaviorMismatch);
    }

    if capability.corruption_model.adaptivity != requirement.corruption_model.adaptivity {
        return MpcEvaluation::incompatible(MpcFailure::CorruptionAdaptivityMismatch);
    }

    if capability.corruption_model.max_corrupt_parties
        < requirement.corruption_model.max_corrupt_parties
    {
        return MpcEvaluation::incompatible(MpcFailure::CorruptionThresholdInsufficient);
    }

    if matches!(requirement.fairness, FairnessRequirement::Required)
        && !matches!(capability.fairness, FairnessDeclaration::DeclaredProvided)
    {
        return MpcEvaluation::incompatible(MpcFailure::FairnessUnavailable);
    }

    if matches!(
        requirement.output_delivery,
        OutputDeliveryRequirement::GuaranteedOutputRequired
    ) && !matches!(
        capability.output_delivery,
        OutputDeliveryDeclaration::DeclaredGuaranteedOutput
    ) {
        return MpcEvaluation::incompatible(MpcFailure::GuaranteedOutputUnavailable);
    }

    if !capability.setup_models.contains(&requirement.setup_model) {
        return MpcEvaluation::incompatible(MpcFailure::SetupModelUnsupported);
    }

    if capability.dropout_model != requirement.dropout_model {
        return MpcEvaluation::incompatible(MpcFailure::DropoutModelMismatch);
    }

    if requirement.computation_id.trim().is_empty() {
        return MpcEvaluation::incompatible(MpcFailure::MissingComputationIdentity);
    }

    if !capability
        .qualified_computation_ids
        .contains(&requirement.computation_id)
    {
        return MpcEvaluation::incompatible(MpcFailure::ComputationIdentityUnsupported);
    }

    MpcEvaluation::compatible()
}

#[cfg(test)]
mod tests {
    use super::*;
    use privacy_computation_core::{
        AdversaryModel, BackendIdentity, InteractionModel, LeakageProfile, LeakageRequirements,
        ParticipantModel, QualificationState,
    };

    fn requirement() -> MpcRequirement {
        MpcRequirement {
            pec: PrivacyRequirement {
                objective: PrivacyObjective::JointInputConfidentiality,
                participant_model: ParticipantModel::MultiParty { participants: 3 },
                adversary_model: AdversaryModel::HonestButCurious,
                interaction_model: InteractionModel::Interactive { rounds: 4 },
                leakage: LeakageRequirements::default(),
                required_qualification: Some(QualificationState::Experimental),
            },
            computation_class: MpcComputationClass::ArithmeticCircuit,
            corruption_model: MpcCorruptionModel {
                behavior: CorruptionBehavior::SemiHonest,
                adaptivity: CorruptionAdaptivity::Static,
                max_corrupt_parties: 1,
            },
            fairness: FairnessRequirement::Unspecified,
            output_delivery: OutputDeliveryRequirement::AbortAllowed,
            setup_model: MpcSetupModel::None,
            dropout_model: DropoutModel::NoDropout,
            computation_id: "sum-v1-digest".into(),
        }
    }

    fn capability() -> MpcCapability {
        MpcCapability {
            pec: PrimitiveCapability {
                backend: BackendIdentity {
                    primitive: PrivacyPrimitive::MultiPartyComputation,
                    backend: "synthetic-mpc".into(),
                    version: "0.0.1".into(),
                    profile: "three-party-demo".into(),
                },
                supported_objectives: vec![PrivacyObjective::JointInputConfidentiality],
                participant_model: ParticipantModel::MultiParty { participants: 3 },
                adversary_model: AdversaryModel::HonestButCurious,
                interaction_model: InteractionModel::Interactive { rounds: 4 },
                leakage: LeakageProfile::default(),
                qualification: QualificationState::Experimental,
            },
            computation_classes: vec![MpcComputationClass::ArithmeticCircuit],
            corruption_model: MpcCorruptionModel {
                behavior: CorruptionBehavior::SemiHonest,
                adaptivity: CorruptionAdaptivity::Static,
                max_corrupt_parties: 1,
            },
            fairness: FairnessDeclaration::NotProvided,
            output_delivery: OutputDeliveryDeclaration::AbortMayPreventOutput,
            setup_models: vec![MpcSetupModel::None],
            dropout_model: DropoutModel::NoDropout,
            qualified_computation_ids: vec!["sum-v1-digest".into()],
        }
    }

    #[test]
    fn compatible_remains_structural_only() {
        let result = evaluate_mpc(&requirement(), &capability());
        assert_eq!(result.disposition, MpcDisposition::Compatible);
        assert_eq!(result.authority, SemanticAuthority::StructuralOnly);
        assert!(!result.input_privacy_established());
        assert!(!result.malicious_security_established());
        assert!(!result.fairness_established());
        assert!(!result.guaranteed_output_established());
        assert!(!result.application_authority_granted());
    }

    #[test]
    fn secure_aggregation_does_not_substitute_for_arithmetic_circuit() {
        let mut cap = capability();
        cap.computation_classes = vec![MpcComputationClass::SecureAggregation];
        assert_eq!(
            evaluate_mpc(&requirement(), &cap).disposition,
            MpcDisposition::Incompatible(MpcFailure::UnsupportedComputationClass)
        );
    }

    #[test]
    fn semi_honest_does_not_substitute_for_malicious() {
        let mut req = requirement();
        req.corruption_model.behavior = CorruptionBehavior::Malicious;
        assert_eq!(
            evaluate_mpc(&req, &capability()).disposition,
            MpcDisposition::Incompatible(MpcFailure::CorruptionBehaviorMismatch)
        );
    }

    #[test]
    fn static_does_not_substitute_for_adaptive() {
        let mut req = requirement();
        req.corruption_model.adaptivity = CorruptionAdaptivity::Adaptive;
        assert_eq!(
            evaluate_mpc(&req, &capability()).disposition,
            MpcDisposition::Incompatible(MpcFailure::CorruptionAdaptivityMismatch)
        );
    }

    #[test]
    fn fairness_is_not_implied_by_mpc() {
        let mut req = requirement();
        req.fairness = FairnessRequirement::Required;
        assert_eq!(
            evaluate_mpc(&req, &capability()).disposition,
            MpcDisposition::Incompatible(MpcFailure::FairnessUnavailable)
        );
    }

    #[test]
    fn guaranteed_output_is_not_implied_by_correctness() {
        let mut req = requirement();
        req.output_delivery = OutputDeliveryRequirement::GuaranteedOutputRequired;
        assert_eq!(
            evaluate_mpc(&req, &capability()).disposition,
            MpcDisposition::Incompatible(MpcFailure::GuaranteedOutputUnavailable)
        );
    }

    #[test]
    fn exact_computation_identity_is_required() {
        let mut req = requirement();
        req.computation_id.clear();
        assert_eq!(
            evaluate_mpc(&req, &capability()).disposition,
            MpcDisposition::Incompatible(MpcFailure::MissingComputationIdentity)
        );
    }

    #[test]
    fn dkg_named_backend_does_not_mint_mpc_security() {
        let mut cap = capability();
        cap.pec.backend.backend = "feldman-dkg-general-mpc".into();
        let result = evaluate_mpc(&requirement(), &cap);
        assert_eq!(result.disposition, MpcDisposition::Compatible);
        assert!(!result.input_privacy_established());
        assert!(!result.malicious_security_established());
    }

    #[test]
    fn core_wire_names_are_stable() {
        assert_eq!(
            serde_json::to_string(&MpcComputationClass::SecureAggregation).unwrap(),
            "\"SecureAggregation\""
        );
        assert_eq!(
            serde_json::to_string(&CorruptionBehavior::Malicious).unwrap(),
            "\"Malicious\""
        );
    }
}
