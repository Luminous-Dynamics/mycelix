// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Authority-neutral private-set semantics layered over Mycelix PEC.
//!
//! This crate implements no PSI protocol and no cryptography. It binds
//! set-specific functionality and disclosure requirements to the existing
//! `privacy-computation-core` structural evaluator.
//!
//! Governing boundary:
//!
//! ```text
//! private-set requirement structurally compatible
//!     != PSI security established
//!     != enumeration resistance established
//!     != dataset comparison authorized
//!     != result disclosure authorized
//! ```

use privacy_computation_core::{
    evaluate_capability, CompositionFailure, CompositionDisposition, PrimitiveCapability,
    PrivacyObjective, PrivacyPrimitive, PrivacyRequirement, SemanticAuthority,
};
use serde::{Deserialize, Serialize};

/// Closed PSI-001A operation vocabulary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PrivateSetOperation {
    Intersection,
    IntersectionCardinality,
    IntersectionAggregate,
}

/// Input collection semantics that must be bound explicitly.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CollectionSemantics {
    Set,
    Multiset,
    OrderedList,
}

/// Duplicate treatment within the bound collection semantics.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DuplicateSemantics {
    RejectDuplicates,
    DeduplicateBeforeProtocol,
    PreserveMultiplicity,
}

/// Exact information the caller permits the private-set operation to disclose.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ResultDisclosure {
    MatchingElements,
    CardinalityOnly,
    AggregateOnly,
}

impl PrivateSetOperation {
    pub const fn required_disclosure(self) -> ResultDisclosure {
        match self {
            Self::Intersection => ResultDisclosure::MatchingElements,
            Self::IntersectionCardinality => ResultDisclosure::CardinalityOnly,
            Self::IntersectionAggregate => ResultDisclosure::AggregateOnly,
        }
    }
}

/// Direction of protocol output disclosure.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum OutputRecipient {
    InitiatorOnly,
    ResponderOnly,
    BothParties,
}

/// Coarse input-domain classification used to prevent low-entropy identifiers
/// from being treated like opaque random set elements.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum IdentifierDomain {
    OpaqueHighEntropy,
    LowEntropyHumanIdentifier,
    ArbitraryApplicationValue,
}

/// Caller requirement for resistance to offline/online identifier enumeration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EnumerationProtectionRequirement {
    Unspecified,
    Required,
}

/// What a profile declares about identifier-enumeration protection.
///
/// `DeclaredProvided` is still only a profile declaration. Qualification of a
/// concrete protocol/backend belongs to an independent evidence layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EnumerationProtectionDeclaration {
    Unspecified,
    NotProvided,
    DeclaredProvided,
}

/// Application-facing private-set requirement.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrivateSetRequirement {
    /// Cross-cutting PEC requirements (adversary, interaction, leakage, etc.).
    pub pec: PrivacyRequirement,
    pub operation: PrivateSetOperation,
    pub collection_semantics: CollectionSemantics,
    pub duplicate_semantics: DuplicateSemantics,
    pub output_recipient: OutputRecipient,
    pub identifier_domain: IdentifierDomain,
    pub enumeration_protection: EnumerationProtectionRequirement,
}

/// Backend/profile declaration for one private-set capability family.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrivateSetCapability {
    /// Cross-cutting PEC capability declaration.
    pub pec: PrimitiveCapability,
    pub operations: Vec<PrivateSetOperation>,
    pub collection_semantics: CollectionSemantics,
    pub duplicate_semantics: DuplicateSemantics,
    pub output_recipient: OutputRecipient,
    pub identifier_domains: Vec<IdentifierDomain>,
    pub enumeration_protection: EnumerationProtectionDeclaration,
}

/// Private-set-specific structural incompatibilities.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PrivateSetFailure {
    WrongPrimitive,
    WrongObjective,
    PecIncompatible(CompositionFailure),
    UnsupportedOperation,
    CollectionSemanticsMismatch,
    DuplicateSemanticsMismatch,
    ResultDisclosureMismatch,
    OutputRecipientMismatch,
    IdentifierDomainUnsupported,
    EnumerationProtectionUnavailable,
}

/// Closed private-set structural disposition.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PrivateSetDisposition {
    Compatible,
    Incompatible(PrivateSetFailure),
}

/// Structural result only; no cryptographic or application authority is minted.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrivateSetEvaluation {
    pub disposition: PrivateSetDisposition,
    pub authority: SemanticAuthority,
}

impl PrivateSetEvaluation {
    fn compatible() -> Self {
        Self {
            disposition: PrivateSetDisposition::Compatible,
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    fn incompatible(reason: PrivateSetFailure) -> Self {
        Self {
            disposition: PrivateSetDisposition::Incompatible(reason),
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    pub const fn psi_security_established(&self) -> bool {
        false
    }

    pub const fn enumeration_resistance_established(&self) -> bool {
        false
    }

    pub const fn dataset_comparison_authorized(&self) -> bool {
        false
    }

    pub const fn result_disclosure_authorized(&self) -> bool {
        false
    }
}

/// Evaluate private-set structural compatibility without selecting or invoking a
/// protocol backend.
pub fn evaluate_private_set(
    requirement: &PrivateSetRequirement,
    capability: &PrivateSetCapability,
) -> PrivateSetEvaluation {
    if capability.pec.backend.primitive != PrivacyPrimitive::PrivateSetOperation {
        return PrivateSetEvaluation::incompatible(PrivateSetFailure::WrongPrimitive);
    }

    if requirement.pec.objective != PrivacyObjective::PrivateSetRelation {
        return PrivateSetEvaluation::incompatible(PrivateSetFailure::WrongObjective);
    }

    let pec_evaluation = evaluate_capability(&requirement.pec, &capability.pec);
    if let CompositionDisposition::Incompatible(reason) = pec_evaluation.disposition {
        return PrivateSetEvaluation::incompatible(PrivateSetFailure::PecIncompatible(reason));
    }

    if !capability.operations.contains(&requirement.operation) {
        return PrivateSetEvaluation::incompatible(PrivateSetFailure::UnsupportedOperation);
    }

    if capability.collection_semantics != requirement.collection_semantics {
        return PrivateSetEvaluation::incompatible(
            PrivateSetFailure::CollectionSemanticsMismatch,
        );
    }

    if capability.duplicate_semantics != requirement.duplicate_semantics {
        return PrivateSetEvaluation::incompatible(PrivateSetFailure::DuplicateSemanticsMismatch);
    }

    // Output disclosure is derived from the exact requested operation. A backend
    // that can reveal *more* information is not a privacy-preserving substitute.
    let required_disclosure = requirement.operation.required_disclosure();
    let capability_discloses_exactly = capability
        .operations
        .iter()
        .any(|op| *op == requirement.operation && op.required_disclosure() == required_disclosure);
    if !capability_discloses_exactly {
        return PrivateSetEvaluation::incompatible(PrivateSetFailure::ResultDisclosureMismatch);
    }

    if capability.output_recipient != requirement.output_recipient {
        return PrivateSetEvaluation::incompatible(PrivateSetFailure::OutputRecipientMismatch);
    }

    if !capability
        .identifier_domains
        .contains(&requirement.identifier_domain)
    {
        return PrivateSetEvaluation::incompatible(PrivateSetFailure::IdentifierDomainUnsupported);
    }

    if matches!(
        requirement.enumeration_protection,
        EnumerationProtectionRequirement::Required
    ) && !matches!(
        capability.enumeration_protection,
        EnumerationProtectionDeclaration::DeclaredProvided
    ) {
        return PrivateSetEvaluation::incompatible(
            PrivateSetFailure::EnumerationProtectionUnavailable,
        );
    }

    PrivateSetEvaluation::compatible()
}

#[cfg(test)]
mod tests {
    use super::*;
    use privacy_computation_core::{
        AdversaryModel, BackendIdentity, DisclosureRequirement, InteractionModel,
        LeakageDeclaration, LeakageProfile, LeakageRequirements, ParticipantModel,
        QualificationState,
    };

    fn base_requirement(operation: PrivateSetOperation) -> PrivateSetRequirement {
        PrivateSetRequirement {
            pec: PrivacyRequirement {
                objective: PrivacyObjective::PrivateSetRelation,
                participant_model: ParticipantModel::TwoParty,
                adversary_model: AdversaryModel::HonestButCurious,
                interaction_model: InteractionModel::Interactive { rounds: 2 },
                leakage: LeakageRequirements {
                    identity: DisclosureRequirement::MustHide,
                    ..LeakageRequirements::default()
                },
                required_qualification: Some(QualificationState::Experimental),
            },
            operation,
            collection_semantics: CollectionSemantics::Set,
            duplicate_semantics: DuplicateSemantics::RejectDuplicates,
            output_recipient: OutputRecipient::InitiatorOnly,
            identifier_domain: IdentifierDomain::OpaqueHighEntropy,
            enumeration_protection: EnumerationProtectionRequirement::Unspecified,
        }
    }

    fn base_capability(operations: Vec<PrivateSetOperation>) -> PrivateSetCapability {
        PrivateSetCapability {
            pec: PrimitiveCapability {
                backend: BackendIdentity {
                    primitive: PrivacyPrimitive::PrivateSetOperation,
                    backend: "synthetic-private-set".into(),
                    version: "0.0.1".into(),
                    profile: "two-party-experimental".into(),
                },
                supported_objectives: vec![PrivacyObjective::PrivateSetRelation],
                participant_model: ParticipantModel::TwoParty,
                adversary_model: AdversaryModel::HonestButCurious,
                interaction_model: InteractionModel::Interactive { rounds: 2 },
                leakage: LeakageProfile {
                    identity: LeakageDeclaration::DeclaredHidden,
                    ..LeakageProfile::default()
                },
                qualification: QualificationState::Experimental,
            },
            operations,
            collection_semantics: CollectionSemantics::Set,
            duplicate_semantics: DuplicateSemantics::RejectDuplicates,
            output_recipient: OutputRecipient::InitiatorOnly,
            identifier_domains: vec![
                IdentifierDomain::OpaqueHighEntropy,
                IdentifierDomain::LowEntropyHumanIdentifier,
            ],
            enumeration_protection: EnumerationProtectionDeclaration::NotProvided,
        }
    }

    #[test]
    fn structural_compatibility_grants_no_authority() {
        let requirement = base_requirement(PrivateSetOperation::Intersection);
        let capability = base_capability(vec![PrivateSetOperation::Intersection]);
        let result = evaluate_private_set(&requirement, &capability);

        assert_eq!(result.disposition, PrivateSetDisposition::Compatible);
        assert_eq!(result.authority, SemanticAuthority::StructuralOnly);
        assert!(!result.psi_security_established());
        assert!(!result.enumeration_resistance_established());
        assert!(!result.dataset_comparison_authorized());
        assert!(!result.result_disclosure_authorized());
    }

    #[test]
    fn cardinality_cannot_fall_back_to_element_revealing_intersection() {
        let requirement = base_requirement(PrivateSetOperation::IntersectionCardinality);
        let capability = base_capability(vec![PrivateSetOperation::Intersection]);

        assert_eq!(
            evaluate_private_set(&requirement, &capability).disposition,
            PrivateSetDisposition::Incompatible(PrivateSetFailure::UnsupportedOperation)
        );
    }

    #[test]
    fn low_entropy_contact_discovery_can_require_enumeration_protection() {
        let mut requirement = base_requirement(PrivateSetOperation::Intersection);
        requirement.identifier_domain = IdentifierDomain::LowEntropyHumanIdentifier;
        requirement.enumeration_protection = EnumerationProtectionRequirement::Required;

        let capability = base_capability(vec![PrivateSetOperation::Intersection]);
        assert_eq!(
            evaluate_private_set(&requirement, &capability).disposition,
            PrivateSetDisposition::Incompatible(
                PrivateSetFailure::EnumerationProtectionUnavailable
            )
        );
    }

    #[test]
    fn an_oprf_named_backend_does_not_mint_enumeration_resistance() {
        let mut requirement = base_requirement(PrivateSetOperation::Intersection);
        requirement.identifier_domain = IdentifierDomain::LowEntropyHumanIdentifier;
        requirement.enumeration_protection = EnumerationProtectionRequirement::Required;

        let mut capability = base_capability(vec![PrivateSetOperation::Intersection]);
        capability.pec.backend.backend = "oprf-shaped-demo".into();

        assert_eq!(
            evaluate_private_set(&requirement, &capability).disposition,
            PrivateSetDisposition::Incompatible(
                PrivateSetFailure::EnumerationProtectionUnavailable
            )
        );
    }

    #[test]
    fn output_recipient_is_exact_not_widened() {
        let requirement = base_requirement(PrivateSetOperation::Intersection);
        let mut capability = base_capability(vec![PrivateSetOperation::Intersection]);
        capability.output_recipient = OutputRecipient::BothParties;

        assert_eq!(
            evaluate_private_set(&requirement, &capability).disposition,
            PrivateSetDisposition::Incompatible(PrivateSetFailure::OutputRecipientMismatch)
        );
    }

    #[test]
    fn pec_leakage_failure_remains_visible() {
        let requirement = base_requirement(PrivateSetOperation::Intersection);
        let mut capability = base_capability(vec![PrivateSetOperation::Intersection]);
        capability.pec.leakage.identity = LeakageDeclaration::MayReveal;

        assert!(matches!(
            evaluate_private_set(&requirement, &capability).disposition,
            PrivateSetDisposition::Incompatible(PrivateSetFailure::PecIncompatible(
                CompositionFailure::LeakageRequirementUnsatisfied(_)
            ))
        ));
    }

    #[test]
    fn core_wire_names_are_stable() {
        assert_eq!(
            serde_json::to_string(&PrivateSetOperation::IntersectionCardinality).unwrap(),
            "\"IntersectionCardinality\""
        );
        assert_eq!(
            serde_json::to_string(&IdentifierDomain::LowEntropyHumanIdentifier).unwrap(),
            "\"LowEntropyHumanIdentifier\""
        );
    }
}
