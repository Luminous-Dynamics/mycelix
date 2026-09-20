// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Authority-neutral PIR semantics layered over Mycelix PEC.
//!
//! This crate implements no PIR protocol and no cryptography. It distinguishes
//! query-index privacy from anonymity, sequence/access-pattern privacy,
//! response integrity, database freshness, and application authorization.

use privacy_computation_core::{
    evaluate_capability, CompositionFailure, CompositionDisposition, PrimitiveCapability,
    PrivacyObjective, PrivacyPrimitive, PrivacyRequirement, SemanticAuthority,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PirServerModel {
    SingleServer,
    MultiServer {
        servers: u16,
        max_colluding_servers: u16,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum QueryDomain {
    RecordIndex,
    EncodedKey,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum RecordLayout {
    FixedSize { bytes: u32 },
    PaddedTo { bytes: u32 },
    VariableSize,
}

/// Privacy requested across one or more retrievals.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SequencePrivacyRequirement {
    SingleQueryOnly,
    RepeatedAccessPatternMustHide,
}

/// What the concrete profile declares about repeated-query privacy.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SequencePrivacyDeclaration {
    SingleQueryOnly,
    DeclaredRepeatedAccessPatternHidden,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ResponseIntegrityRequirement {
    Unspecified,
    Required,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ResponseIntegrityDeclaration {
    Unspecified,
    NotProvided,
    DeclaredProvided,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SnapshotFreshnessRequirement {
    Unspecified,
    ExactSnapshotRequired,
    CurrentAtVerificationRequired,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SnapshotFreshnessDeclaration {
    Unspecified,
    ExactSnapshotBound,
    CurrentAtVerificationDeclared,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PirRequirement {
    pub pec: PrivacyRequirement,
    pub server_model: PirServerModel,
    pub query_domain: QueryDomain,
    pub record_layout: RecordLayout,
    pub sequence_privacy: SequencePrivacyRequirement,
    pub response_integrity: ResponseIntegrityRequirement,
    pub snapshot_freshness: SnapshotFreshnessRequirement,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PirCapability {
    pub pec: PrimitiveCapability,
    pub server_model: PirServerModel,
    pub query_domains: Vec<QueryDomain>,
    pub record_layouts: Vec<RecordLayout>,
    pub sequence_privacy: SequencePrivacyDeclaration,
    pub response_integrity: ResponseIntegrityDeclaration,
    pub snapshot_freshness: SnapshotFreshnessDeclaration,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PirFailure {
    WrongPrimitive,
    WrongObjective,
    PecIncompatible(CompositionFailure),
    ServerModelMismatch,
    QueryDomainUnsupported,
    RecordLayoutUnsupported,
    AccessPatternPrivacyUnavailable,
    ResponseIntegrityUnavailable,
    SnapshotFreshnessUnavailable,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PirDisposition {
    Compatible,
    Incompatible(PirFailure),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PirEvaluation {
    pub disposition: PirDisposition,
    pub authority: SemanticAuthority,
}

impl PirEvaluation {
    fn compatible() -> Self {
        Self {
            disposition: PirDisposition::Compatible,
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    fn incompatible(reason: PirFailure) -> Self {
        Self {
            disposition: PirDisposition::Incompatible(reason),
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    pub const fn query_privacy_established(&self) -> bool {
        false
    }

    pub const fn client_anonymity_established(&self) -> bool {
        false
    }

    pub const fn access_pattern_privacy_established(&self) -> bool {
        false
    }

    pub const fn response_integrity_established(&self) -> bool {
        false
    }

    pub const fn snapshot_freshness_established(&self) -> bool {
        false
    }

    pub const fn retrieval_authorized(&self) -> bool {
        false
    }
}

pub fn evaluate_pir(
    requirement: &PirRequirement,
    capability: &PirCapability,
) -> PirEvaluation {
    if capability.pec.backend.primitive != PrivacyPrimitive::PrivateInformationRetrieval {
        return PirEvaluation::incompatible(PirFailure::WrongPrimitive);
    }

    if requirement.pec.objective != PrivacyObjective::QueryIndexPrivacy {
        return PirEvaluation::incompatible(PirFailure::WrongObjective);
    }

    let pec = evaluate_capability(&requirement.pec, &capability.pec);
    if let CompositionDisposition::Incompatible(reason) = pec.disposition {
        return PirEvaluation::incompatible(PirFailure::PecIncompatible(reason));
    }

    if capability.server_model != requirement.server_model {
        return PirEvaluation::incompatible(PirFailure::ServerModelMismatch);
    }

    if !capability.query_domains.contains(&requirement.query_domain) {
        return PirEvaluation::incompatible(PirFailure::QueryDomainUnsupported);
    }

    if !capability.record_layouts.contains(&requirement.record_layout) {
        return PirEvaluation::incompatible(PirFailure::RecordLayoutUnsupported);
    }

    if matches!(
        requirement.sequence_privacy,
        SequencePrivacyRequirement::RepeatedAccessPatternMustHide
    ) && !matches!(
        capability.sequence_privacy,
        SequencePrivacyDeclaration::DeclaredRepeatedAccessPatternHidden
    ) {
        return PirEvaluation::incompatible(PirFailure::AccessPatternPrivacyUnavailable);
    }

    if matches!(
        requirement.response_integrity,
        ResponseIntegrityRequirement::Required
    ) && !matches!(
        capability.response_integrity,
        ResponseIntegrityDeclaration::DeclaredProvided
    ) {
        return PirEvaluation::incompatible(PirFailure::ResponseIntegrityUnavailable);
    }

    match requirement.snapshot_freshness {
        SnapshotFreshnessRequirement::Unspecified => {}
        SnapshotFreshnessRequirement::ExactSnapshotRequired => {
            if !matches!(
                capability.snapshot_freshness,
                SnapshotFreshnessDeclaration::ExactSnapshotBound
                    | SnapshotFreshnessDeclaration::CurrentAtVerificationDeclared
            ) {
                return PirEvaluation::incompatible(PirFailure::SnapshotFreshnessUnavailable);
            }
        }
        SnapshotFreshnessRequirement::CurrentAtVerificationRequired => {
            if !matches!(
                capability.snapshot_freshness,
                SnapshotFreshnessDeclaration::CurrentAtVerificationDeclared
            ) {
                return PirEvaluation::incompatible(PirFailure::SnapshotFreshnessUnavailable);
            }
        }
    }

    PirEvaluation::compatible()
}

#[cfg(test)]
mod tests {
    use super::*;
    use privacy_computation_core::{
        AdversaryModel, BackendIdentity, DisclosureRequirement, InteractionModel,
        LeakageDeclaration, LeakageProfile, LeakageRequirements, ParticipantModel,
        QualificationState,
    };

    fn requirement() -> PirRequirement {
        PirRequirement {
            pec: PrivacyRequirement {
                objective: PrivacyObjective::QueryIndexPrivacy,
                participant_model: ParticipantModel::TwoParty,
                adversary_model: AdversaryModel::HonestButCurious,
                interaction_model: InteractionModel::Interactive { rounds: 1 },
                leakage: LeakageRequirements {
                    query_index: DisclosureRequirement::MustHide,
                    ..LeakageRequirements::default()
                },
                required_qualification: Some(QualificationState::Experimental),
            },
            server_model: PirServerModel::SingleServer,
            query_domain: QueryDomain::RecordIndex,
            record_layout: RecordLayout::FixedSize { bytes: 1024 },
            sequence_privacy: SequencePrivacyRequirement::SingleQueryOnly,
            response_integrity: ResponseIntegrityRequirement::Unspecified,
            snapshot_freshness: SnapshotFreshnessRequirement::Unspecified,
        }
    }

    fn capability() -> PirCapability {
        PirCapability {
            pec: PrimitiveCapability {
                backend: BackendIdentity {
                    primitive: PrivacyPrimitive::PrivateInformationRetrieval,
                    backend: "synthetic-pir".into(),
                    version: "0.0.1".into(),
                    profile: "single-server-demo".into(),
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
            },
            server_model: PirServerModel::SingleServer,
            query_domains: vec![QueryDomain::RecordIndex],
            record_layouts: vec![RecordLayout::FixedSize { bytes: 1024 }],
            sequence_privacy: SequencePrivacyDeclaration::SingleQueryOnly,
            response_integrity: ResponseIntegrityDeclaration::NotProvided,
            snapshot_freshness: SnapshotFreshnessDeclaration::Unspecified,
        }
    }

    #[test]
    fn compatible_remains_non_authoritative() {
        let result = evaluate_pir(&requirement(), &capability());
        assert_eq!(result.disposition, PirDisposition::Compatible);
        assert_eq!(result.authority, SemanticAuthority::StructuralOnly);
        assert!(!result.query_privacy_established());
        assert!(!result.client_anonymity_established());
        assert!(!result.access_pattern_privacy_established());
        assert!(!result.response_integrity_established());
        assert!(!result.snapshot_freshness_established());
        assert!(!result.retrieval_authorized());
    }

    #[test]
    fn repeated_access_pattern_requirement_rejects_single_query_pir() {
        let mut req = requirement();
        req.sequence_privacy = SequencePrivacyRequirement::RepeatedAccessPatternMustHide;
        assert_eq!(
            evaluate_pir(&req, &capability()).disposition,
            PirDisposition::Incompatible(PirFailure::AccessPatternPrivacyUnavailable)
        );
    }

    #[test]
    fn integrity_is_not_implied_by_private_retrieval() {
        let mut req = requirement();
        req.response_integrity = ResponseIntegrityRequirement::Required;
        assert_eq!(
            evaluate_pir(&req, &capability()).disposition,
            PirDisposition::Incompatible(PirFailure::ResponseIntegrityUnavailable)
        );
    }

    #[test]
    fn current_freshness_is_stricter_than_exact_snapshot_binding() {
        let mut req = requirement();
        req.snapshot_freshness = SnapshotFreshnessRequirement::CurrentAtVerificationRequired;
        let mut cap = capability();
        cap.snapshot_freshness = SnapshotFreshnessDeclaration::ExactSnapshotBound;
        assert_eq!(
            evaluate_pir(&req, &cap).disposition,
            PirDisposition::Incompatible(PirFailure::SnapshotFreshnessUnavailable)
        );
    }

    #[test]
    fn single_server_does_not_substitute_for_multi_server_model() {
        let mut req = requirement();
        req.server_model = PirServerModel::MultiServer {
            servers: 2,
            max_colluding_servers: 0,
        };
        assert_eq!(
            evaluate_pir(&req, &capability()).disposition,
            PirDisposition::Incompatible(PirFailure::ServerModelMismatch)
        );
    }

    #[test]
    fn pir_named_backend_does_not_mint_anonymity() {
        let mut cap = capability();
        cap.pec.backend.backend = "anonymous-super-pir".into();
        let result = evaluate_pir(&requirement(), &cap);
        assert_eq!(result.disposition, PirDisposition::Compatible);
        assert!(!result.client_anonymity_established());
    }

    #[test]
    fn core_wire_names_are_stable() {
        assert_eq!(
            serde_json::to_string(&SequencePrivacyRequirement::RepeatedAccessPatternMustHide)
                .unwrap(),
            "\"RepeatedAccessPatternMustHide\""
        );
        assert_eq!(
            serde_json::to_string(&PirServerModel::SingleServer).unwrap(),
            "\"SingleServer\""
        );
    }
}
