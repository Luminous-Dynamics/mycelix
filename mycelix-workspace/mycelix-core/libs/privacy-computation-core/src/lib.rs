// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Authority-neutral semantic types for privacy-enhancing computation (PEC).
//!
//! This crate deliberately implements no cryptography. It describes privacy
//! requirements, backend declarations, and structural compatibility only.
//!
//! Governing boundary:
//!
//! ```text
//! requirement structurally compatible
//!     != privacy property established
//!     != backend qualified
//!     != composition qualified
//!     != application authority
//! ```

use serde::{Deserialize, Serialize};

/// Closed PEC-000A primitive vocabulary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PrivacyPrimitive {
    ZeroKnowledge,
    MultiPartyComputation,
    HomomorphicEncryption,
    PrivateSetOperation,
    PrivateInformationRetrieval,
    DifferentialPrivacy,
}

/// Closed PEC-000A privacy-objective vocabulary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PrivacyObjective {
    WitnessConfidentiality,
    JointInputConfidentiality,
    ComputeOnCiphertext,
    PrivateSetRelation,
    QueryIndexPrivacy,
    StatisticalDisclosureLimitation,
}

/// A caller's disclosure requirement for one leakage dimension.
///
/// `MustHide` is a requested property, not evidence that it is achieved.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DisclosureRequirement {
    Unspecified,
    MayReveal,
    MustHide,
}

/// What a capability profile *declares* about one leakage dimension.
///
/// `DeclaredHidden` remains a declaration. Cryptographic qualification belongs
/// to a separate evidence/admission layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LeakageDeclaration {
    Unspecified,
    MayReveal,
    DeclaredHidden,
}

/// Leakage dimensions intentionally kept explicit rather than collapsed into a
/// `private: bool` flag.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LeakageDimension {
    Identity,
    ParticipantCount,
    InputSize,
    SetSize,
    QueryIndex,
    AccessPattern,
    OutputValue,
}

/// Disclosure requirements across common leakage dimensions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LeakageRequirements {
    pub identity: DisclosureRequirement,
    pub participant_count: DisclosureRequirement,
    pub input_size: DisclosureRequirement,
    pub set_size: DisclosureRequirement,
    pub query_index: DisclosureRequirement,
    pub access_pattern: DisclosureRequirement,
    pub output_value: DisclosureRequirement,
}

impl Default for LeakageRequirements {
    fn default() -> Self {
        Self {
            identity: DisclosureRequirement::Unspecified,
            participant_count: DisclosureRequirement::Unspecified,
            input_size: DisclosureRequirement::Unspecified,
            set_size: DisclosureRequirement::Unspecified,
            query_index: DisclosureRequirement::Unspecified,
            access_pattern: DisclosureRequirement::Unspecified,
            output_value: DisclosureRequirement::Unspecified,
        }
    }
}

/// A backend/profile's declared leakage surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LeakageProfile {
    pub identity: LeakageDeclaration,
    pub participant_count: LeakageDeclaration,
    pub input_size: LeakageDeclaration,
    pub set_size: LeakageDeclaration,
    pub query_index: LeakageDeclaration,
    pub access_pattern: LeakageDeclaration,
    pub output_value: LeakageDeclaration,
}

impl Default for LeakageProfile {
    fn default() -> Self {
        Self {
            identity: LeakageDeclaration::Unspecified,
            participant_count: LeakageDeclaration::Unspecified,
            input_size: LeakageDeclaration::Unspecified,
            set_size: LeakageDeclaration::Unspecified,
            query_index: LeakageDeclaration::Unspecified,
            access_pattern: LeakageDeclaration::Unspecified,
            output_value: LeakageDeclaration::Unspecified,
        }
    }
}

/// Participant topology required or supported by a profile.
///
/// PEC-001A intentionally evaluates this by exact match. More permissive
/// topology-subtyping requires a later, separately frozen theorem.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ParticipantModel {
    SingleParty,
    TwoParty,
    MultiParty { participants: u16 },
}

/// Adversary model requested or declared by a profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AdversaryModel {
    Unspecified,
    HonestButCurious,
    Malicious,
}

/// Interaction shape requested or declared by a profile.
///
/// Exact-match semantics avoid silently treating a more interactive protocol
/// as satisfying a non-interactive requirement.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum InteractionModel {
    NonInteractive,
    Interactive { rounds: u32 },
}

/// PEC-000A qualification vocabulary.
///
/// Deliberately does not implement `Ord`/`PartialOrd`: variant order is not an
/// evidence ladder and must not be used to infer qualification.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum QualificationState {
    SimulationOnly,
    Experimental,
    Measured,
    Qualified,
    ProductionAdmitted,
}

impl QualificationState {
    /// True only for an explicitly separate production-admission disposition.
    pub const fn is_production_admitted(self) -> bool {
        matches!(self, Self::ProductionAdmitted)
    }
}

/// Stable identity for a concrete backend/profile declaration.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BackendIdentity {
    pub primitive: PrivacyPrimitive,
    pub backend: String,
    pub version: String,
    pub profile: String,
}

/// Structural privacy requirement supplied by a caller/planner.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrivacyRequirement {
    pub objective: PrivacyObjective,
    pub participant_model: ParticipantModel,
    pub adversary_model: AdversaryModel,
    pub interaction_model: InteractionModel,
    pub leakage: LeakageRequirements,
    /// If present, PEC-001A requires an exact qualification-state match.
    /// No implicit ordering or promotion is permitted.
    pub required_qualification: Option<QualificationState>,
}

/// Backend/profile declaration consumed by the structural evaluator.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrimitiveCapability {
    pub backend: BackendIdentity,
    pub supported_objectives: Vec<PrivacyObjective>,
    pub participant_model: ParticipantModel,
    pub adversary_model: AdversaryModel,
    pub interaction_model: InteractionModel,
    pub leakage: LeakageProfile,
    pub qualification: QualificationState,
}

/// Reasons a requirement/profile composition is structurally refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CompositionFailure {
    UnsupportedObjective,
    ParticipantModelMismatch,
    AdversaryModelMismatch,
    InteractionModelMismatch,
    QualificationMismatch,
    LeakageRequirementUnsatisfied(LeakageDimension),
}

/// Closed structural disposition. There is no fallback-to-nearest-primitive
/// state: unsupported combinations fail closed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CompositionDisposition {
    Compatible,
    Incompatible(CompositionFailure),
}

/// Machine-readable authority ceiling for this crate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SemanticAuthority {
    StructuralOnly,
}

/// Result returned by the structural compatibility evaluator.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilityEvaluation {
    pub disposition: CompositionDisposition,
    pub authority: SemanticAuthority,
}

impl CapabilityEvaluation {
    fn new(disposition: CompositionDisposition) -> Self {
        Self {
            disposition,
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    /// Structural compatibility never proves a privacy property.
    pub const fn privacy_established(&self) -> bool {
        false
    }

    /// Structural compatibility never grants production admission.
    pub const fn production_admission_granted(&self) -> bool {
        false
    }

    /// Structural compatibility never grants application authority.
    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

fn disclosure_satisfied(
    requirement: DisclosureRequirement,
    declaration: LeakageDeclaration,
) -> bool {
    match requirement {
        DisclosureRequirement::Unspecified | DisclosureRequirement::MayReveal => true,
        DisclosureRequirement::MustHide => matches!(declaration, LeakageDeclaration::DeclaredHidden),
    }
}

fn first_leakage_failure(
    requirement: &LeakageRequirements,
    profile: &LeakageProfile,
) -> Option<LeakageDimension> {
    let dimensions = [
        (
            LeakageDimension::Identity,
            requirement.identity,
            profile.identity,
        ),
        (
            LeakageDimension::ParticipantCount,
            requirement.participant_count,
            profile.participant_count,
        ),
        (
            LeakageDimension::InputSize,
            requirement.input_size,
            profile.input_size,
        ),
        (
            LeakageDimension::SetSize,
            requirement.set_size,
            profile.set_size,
        ),
        (
            LeakageDimension::QueryIndex,
            requirement.query_index,
            profile.query_index,
        ),
        (
            LeakageDimension::AccessPattern,
            requirement.access_pattern,
            profile.access_pattern,
        ),
        (
            LeakageDimension::OutputValue,
            requirement.output_value,
            profile.output_value,
        ),
    ];

    dimensions
        .into_iter()
        .find_map(|(dimension, required, declared)| {
            (!disclosure_satisfied(required, declared)).then_some(dimension)
        })
}

/// Evaluate structural compatibility only.
///
/// This function intentionally performs exact matches for participant,
/// adversary, interaction, and optional qualification state. It does not select
/// a backend, prove privacy, authenticate evidence, or grant authority.
pub fn evaluate_capability(
    requirement: &PrivacyRequirement,
    capability: &PrimitiveCapability,
) -> CapabilityEvaluation {
    if !capability.supported_objectives.contains(&requirement.objective) {
        return CapabilityEvaluation::new(CompositionDisposition::Incompatible(
            CompositionFailure::UnsupportedObjective,
        ));
    }

    if capability.participant_model != requirement.participant_model {
        return CapabilityEvaluation::new(CompositionDisposition::Incompatible(
            CompositionFailure::ParticipantModelMismatch,
        ));
    }

    if capability.adversary_model != requirement.adversary_model {
        return CapabilityEvaluation::new(CompositionDisposition::Incompatible(
            CompositionFailure::AdversaryModelMismatch,
        ));
    }

    if capability.interaction_model != requirement.interaction_model {
        return CapabilityEvaluation::new(CompositionDisposition::Incompatible(
            CompositionFailure::InteractionModelMismatch,
        ));
    }

    if let Some(required) = requirement.required_qualification {
        if capability.qualification != required {
            return CapabilityEvaluation::new(CompositionDisposition::Incompatible(
                CompositionFailure::QualificationMismatch,
            ));
        }
    }

    if let Some(dimension) = first_leakage_failure(&requirement.leakage, &capability.leakage) {
        return CapabilityEvaluation::new(CompositionDisposition::Incompatible(
            CompositionFailure::LeakageRequirementUnsatisfied(dimension),
        ));
    }

    CapabilityEvaluation::new(CompositionDisposition::Compatible)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base_requirement() -> PrivacyRequirement {
        PrivacyRequirement {
            objective: PrivacyObjective::QueryIndexPrivacy,
            participant_model: ParticipantModel::TwoParty,
            adversary_model: AdversaryModel::HonestButCurious,
            interaction_model: InteractionModel::Interactive { rounds: 1 },
            leakage: LeakageRequirements {
                query_index: DisclosureRequirement::MustHide,
                ..LeakageRequirements::default()
            },
            required_qualification: Some(QualificationState::Experimental),
        }
    }

    fn base_capability() -> PrimitiveCapability {
        PrimitiveCapability {
            backend: BackendIdentity {
                primitive: PrivacyPrimitive::PrivateInformationRetrieval,
                backend: "synthetic-pir".into(),
                version: "0.0.1".into(),
                profile: "two-party-demo".into(),
            },
            supported_objectives: vec![PrivacyObjective::QueryIndexPrivacy],
            participant_model: ParticipantModel::TwoParty,
            adversary_model: AdversaryModel::HonestButCurious,
            interaction_model: InteractionModel::Interactive { rounds: 1 },
            leakage: LeakageProfile {
                query_index: LeakageDeclaration::DeclaredHidden,
                ..LeakageProfile::default()
            },
            qualification: QualificationState::Experimental,
        }
    }

    #[test]
    fn simulation_only_is_never_production_admitted() {
        assert!(!QualificationState::SimulationOnly.is_production_admitted());
        assert!(QualificationState::ProductionAdmitted.is_production_admitted());
    }

    #[test]
    fn compatible_is_structural_only() {
        let evaluation = evaluate_capability(&base_requirement(), &base_capability());
        assert_eq!(evaluation.disposition, CompositionDisposition::Compatible);
        assert_eq!(evaluation.authority, SemanticAuthority::StructuralOnly);
        assert!(!evaluation.privacy_established());
        assert!(!evaluation.production_admission_granted());
        assert!(!evaluation.application_authority_granted());
    }

    #[test]
    fn unsupported_objective_fails_closed() {
        let mut requirement = base_requirement();
        requirement.objective = PrivacyObjective::PrivateSetRelation;
        let evaluation = evaluate_capability(&requirement, &base_capability());
        assert_eq!(
            evaluation.disposition,
            CompositionDisposition::Incompatible(CompositionFailure::UnsupportedObjective)
        );
    }

    #[test]
    fn required_hidden_query_rejects_may_reveal() {
        let mut capability = base_capability();
        capability.leakage.query_index = LeakageDeclaration::MayReveal;
        let evaluation = evaluate_capability(&base_requirement(), &capability);
        assert_eq!(
            evaluation.disposition,
            CompositionDisposition::Incompatible(
                CompositionFailure::LeakageRequirementUnsatisfied(LeakageDimension::QueryIndex)
            )
        );
    }

    #[test]
    fn qualification_is_exact_not_ordered() {
        let mut capability = base_capability();
        capability.qualification = QualificationState::ProductionAdmitted;
        let evaluation = evaluate_capability(&base_requirement(), &capability);
        assert_eq!(
            evaluation.disposition,
            CompositionDisposition::Incompatible(CompositionFailure::QualificationMismatch)
        );
    }

    #[test]
    fn participant_model_does_not_implicitly_widen() {
        let mut capability = base_capability();
        capability.participant_model = ParticipantModel::MultiParty { participants: 3 };
        let evaluation = evaluate_capability(&base_requirement(), &capability);
        assert_eq!(
            evaluation.disposition,
            CompositionDisposition::Incompatible(CompositionFailure::ParticipantModelMismatch)
        );
    }

    #[test]
    fn wire_names_are_explicitly_stable_for_core_vocabularies() {
        assert_eq!(
            serde_json::to_string(&PrivacyPrimitive::HomomorphicEncryption).unwrap(),
            "\"HomomorphicEncryption\""
        );
        assert_eq!(
            serde_json::to_string(&QualificationState::SimulationOnly).unwrap(),
            "\"SimulationOnly\""
        );
        assert_eq!(
            serde_json::to_string(&PrivacyObjective::PrivateSetRelation).unwrap(),
            "\"PrivateSetRelation\""
        );
    }
}
