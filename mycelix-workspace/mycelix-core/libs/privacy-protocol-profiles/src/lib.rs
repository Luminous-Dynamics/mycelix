// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Backend-neutral structural profiles for MPC, FHE, PSI, and PIR.
//!
//! This crate implements no cryptography. Structural validity is not proof of
//! backend security, privacy, production admission, or application authority.

use privacy_computation_core::{
    AdversaryModel, BackendIdentity, InteractionModel, LeakageProfile, ParticipantModel,
    PrivacyObjective, PrimitiveCapability, QualificationState,
};
use serde::{Deserialize, Serialize};

pub mod fhe;
pub mod mpc;
pub mod pir;
pub mod psi;

pub use fhe::*;
pub use mpc::*;
pub use pir::*;
pub use psi::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProfileError {
    WrongPrimitive,
    EmptyBackendName,
    EmptyBackendVersion,
    EmptyBackendProfile,
    ZeroRoundInteractive,
    InvalidParticipantModel,
    InvalidCorruptionThreshold,
    InvalidDesignatedRecipient,
    DuplicateDesignatedRecipient,
    MpcOutputDisclosureUnspecified,
    MpcNoOutputDisclosureConflict,
    InvalidThresholdKeyModel,
    InvalidMultiKeyModel,
    FheKeyParticipantMismatch,
    FheAccessPatternOverclaim,
    EmptyPsiEqualityDomain,
    EmptyPsiSessionDomain,
    PsiIntersectionDisclosureUnspecified,
    PsiInconsistentResultLeakage,
    PsiCardinalityRevealsElements,
    PsiAggregateRevealsElements,
    PsiCardinalityDisclosureUnspecified,
    PsiAggregateDisclosureUnspecified,
    PsiAggregateSpecRequired,
    PsiUnexpectedAggregateSpec,
    EmptyPsiAggregateValueDomain,
    EmptyPirDatabaseSnapshot,
    PirQueryIndexNotHidden,
    PirAccessPatternOverclaim,
    InvalidPirServerModel,
    InvalidPirBatchSize,
    PirParticipantTopologyMismatch,
    PirParticipantCountOverflow,
}

pub(crate) fn validate_common(
    backend: &BackendIdentity,
    interaction_model: InteractionModel,
) -> Result<(), ProfileError> {
    if backend.backend.trim().is_empty() {
        return Err(ProfileError::EmptyBackendName);
    }
    if backend.version.trim().is_empty() {
        return Err(ProfileError::EmptyBackendVersion);
    }
    if backend.profile.trim().is_empty() {
        return Err(ProfileError::EmptyBackendProfile);
    }
    if matches!(interaction_model, InteractionModel::Interactive { rounds: 0 }) {
        return Err(ProfileError::ZeroRoundInteractive);
    }
    Ok(())
}

pub(crate) fn participant_count(model: ParticipantModel) -> Option<u16> {
    match model {
        ParticipantModel::SingleParty => Some(1),
        ParticipantModel::TwoParty => Some(2),
        ParticipantModel::MultiParty { participants } if participants >= 2 => Some(participants),
        ParticipantModel::MultiParty { .. } => None,
    }
}

pub(crate) fn capability(
    backend: BackendIdentity,
    objective: PrivacyObjective,
    participant_model: ParticipantModel,
    adversary_model: AdversaryModel,
    interaction_model: InteractionModel,
    leakage: LeakageProfile,
    qualification: QualificationState,
) -> PrimitiveCapability {
    PrimitiveCapability {
        backend,
        supported_objectives: vec![objective],
        participant_model,
        adversary_model,
        interaction_model,
        leakage,
        qualification,
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProtocolProfile {
    Mpc(MpcProfile),
    Fhe(FheProfile),
    Psi(PsiProfile),
    Pir(PirProfile),
}

impl ProtocolProfile {
    pub fn as_capability(&self) -> Result<PrimitiveCapability, ProfileError> {
        match self {
            Self::Mpc(profile) => profile.as_capability(),
            Self::Fhe(profile) => profile.as_capability(),
            Self::Psi(profile) => profile.as_capability(),
            Self::Pir(profile) => profile.as_capability(),
        }
    }

    pub const fn cryptographic_security_established(&self) -> bool {
        false
    }

    pub const fn production_admission_granted(&self) -> bool {
        false
    }

    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use privacy_computation_core::{LeakageDeclaration, PrivacyPrimitive};

    fn backend() -> BackendIdentity {
        BackendIdentity {
            primitive: PrivacyPrimitive::PrivateInformationRetrieval,
            backend: "synthetic".into(),
            version: "0.0.1".into(),
            profile: "structural-test".into(),
        }
    }

    #[test]
    fn common_identity_fields_are_required() {
        let mut id = backend();
        id.backend = "   ".into();
        assert_eq!(
            validate_common(&id, InteractionModel::NonInteractive),
            Err(ProfileError::EmptyBackendName)
        );
        id = backend();
        id.version.clear();
        assert_eq!(
            validate_common(&id, InteractionModel::NonInteractive),
            Err(ProfileError::EmptyBackendVersion)
        );
        id = backend();
        id.profile.clear();
        assert_eq!(
            validate_common(&id, InteractionModel::NonInteractive),
            Err(ProfileError::EmptyBackendProfile)
        );
    }

    #[test]
    fn interactive_zero_rounds_is_rejected() {
        assert_eq!(
            validate_common(&backend(), InteractionModel::Interactive { rounds: 0 }),
            Err(ProfileError::ZeroRoundInteractive)
        );
    }

    #[test]
    fn protocol_wrapper_never_grants_authority() {
        let profile = PirProfile {
            backend: backend(),
            participant_model: ParticipantModel::TwoParty,
            adversary_model: AdversaryModel::HonestButCurious,
            interaction_model: InteractionModel::Interactive { rounds: 1 },
            server_model: PirServerModel::SingleServer,
            query_shape: PirQueryShape::SingleIndex,
            database_snapshot: "public-registry-v1".into(),
            leakage: LeakageProfile {
                query_index: LeakageDeclaration::DeclaredHidden,
                access_pattern: LeakageDeclaration::MayReveal,
                ..LeakageProfile::default()
            },
            repeated_query_linkability: LeakageDeclaration::MayReveal,
            response_size: LeakageDeclaration::MayReveal,
            timing: LeakageDeclaration::MayReveal,
            qualification: QualificationState::Experimental,
        };
        let wrapped = ProtocolProfile::Pir(profile);
        let capability = wrapped.as_capability().unwrap();
        assert_eq!(capability.supported_objectives, vec![PrivacyObjective::QueryIndexPrivacy]);
        assert!(!wrapped.cryptographic_security_established());
        assert!(!wrapped.production_admission_granted());
        assert!(!wrapped.application_authority_granted());
    }
}
