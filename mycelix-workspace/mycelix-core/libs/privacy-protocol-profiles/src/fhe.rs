use crate::{capability, participant_count, validate_common, ProfileError};
use privacy_computation_core::{
    AdversaryModel, BackendIdentity, InteractionModel, LeakageDeclaration, LeakageProfile,
    ParticipantModel, PrivacyObjective, PrivacyPrimitive, PrimitiveCapability, QualificationState,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FheNumericSemantics {
    Boolean,
    ExactInteger,
    ApproximateReal,
}

pub const fn fhe_numeric_compatible(
    required: FheNumericSemantics,
    offered: FheNumericSemantics,
) -> bool {
    matches!(
        (required, offered),
        (FheNumericSemantics::Boolean, FheNumericSemantics::Boolean)
            | (FheNumericSemantics::ExactInteger, FheNumericSemantics::ExactInteger)
            | (FheNumericSemantics::ApproximateReal, FheNumericSemantics::ApproximateReal)
    )
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FheKeyModel {
    SingleKey,
    Threshold { participants: u16, threshold: u16 },
    MultiKey { participants: u16 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FheBootstrappingModel {
    Unsupported,
    Optional,
    Required,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FheExecutionTarget {
    Cpu,
    Gpu,
    Wasm,
    CpuGpu,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FheProfile {
    pub backend: BackendIdentity,
    pub participant_model: ParticipantModel,
    pub adversary_model: AdversaryModel,
    pub interaction_model: InteractionModel,
    pub numeric_semantics: FheNumericSemantics,
    pub key_model: FheKeyModel,
    pub bootstrapping: FheBootstrappingModel,
    pub execution_target: FheExecutionTarget,
    pub max_multiplicative_depth: Option<u32>,
    pub leakage: LeakageProfile,
    pub qualification: QualificationState,
}

impl FheProfile {
    pub fn validate(&self) -> Result<(), ProfileError> {
        if self.backend.primitive != PrivacyPrimitive::HomomorphicEncryption {
            return Err(ProfileError::WrongPrimitive);
        }
        validate_common(&self.backend, self.interaction_model)?;
        let participants = participant_count(self.participant_model)
            .ok_or(ProfileError::InvalidParticipantModel)?;
        match self.key_model {
            FheKeyModel::SingleKey => {}
            FheKeyModel::Threshold {
                participants: key_participants,
                threshold,
            } if key_participants >= 2 && threshold > 0 && threshold <= key_participants => {
                if participants != key_participants {
                    return Err(ProfileError::FheKeyParticipantMismatch);
                }
            }
            FheKeyModel::Threshold { .. } => return Err(ProfileError::InvalidThresholdKeyModel),
            FheKeyModel::MultiKey {
                participants: key_participants,
            } if key_participants >= 2 => {
                if participants != key_participants {
                    return Err(ProfileError::FheKeyParticipantMismatch);
                }
            }
            FheKeyModel::MultiKey { .. } => return Err(ProfileError::InvalidMultiKeyModel),
        }
        if self.leakage.access_pattern == LeakageDeclaration::DeclaredHidden {
            return Err(ProfileError::FheAccessPatternOverclaim);
        }
        Ok(())
    }

    pub fn supports_numeric_semantics(&self, required: FheNumericSemantics) -> bool {
        fhe_numeric_compatible(required, self.numeric_semantics)
    }

    pub fn as_capability(&self) -> Result<PrimitiveCapability, ProfileError> {
        self.validate()?;
        Ok(capability(
            self.backend.clone(),
            PrivacyObjective::ComputeOnCiphertext,
            self.participant_model,
            self.adversary_model,
            self.interaction_model,
            self.leakage,
            self.qualification,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn backend() -> BackendIdentity {
        BackendIdentity {
            primitive: PrivacyPrimitive::HomomorphicEncryption,
            backend: "synthetic".into(),
            version: "0.0.1".into(),
            profile: "structural-test".into(),
        }
    }

    fn base() -> FheProfile {
        FheProfile {
            backend: backend(),
            participant_model: ParticipantModel::TwoParty,
            adversary_model: AdversaryModel::HonestButCurious,
            interaction_model: InteractionModel::NonInteractive,
            numeric_semantics: FheNumericSemantics::ExactInteger,
            key_model: FheKeyModel::SingleKey,
            bootstrapping: FheBootstrappingModel::Optional,
            execution_target: FheExecutionTarget::Cpu,
            max_multiplicative_depth: Some(4),
            leakage: LeakageProfile::default(),
            qualification: QualificationState::Experimental,
        }
    }

    #[test]
    fn approximate_real_never_substitutes_for_exact_integer() {
        assert!(!fhe_numeric_compatible(
            FheNumericSemantics::ExactInteger,
            FheNumericSemantics::ApproximateReal
        ));
    }

    #[test]
    fn plain_fhe_rejects_access_pattern_overclaim() {
        let mut profile = base();
        profile.leakage.access_pattern = LeakageDeclaration::DeclaredHidden;
        assert_eq!(profile.validate(), Err(ProfileError::FheAccessPatternOverclaim));
    }

    #[test]
    fn threshold_key_shape_fails_closed() {
        let mut profile = base();
        profile.participant_model = ParticipantModel::MultiParty { participants: 3 };
        profile.key_model = FheKeyModel::Threshold { participants: 3, threshold: 4 };
        assert_eq!(profile.validate(), Err(ProfileError::InvalidThresholdKeyModel));
    }

    #[test]
    fn threshold_key_party_count_must_match_participant_topology() {
        let mut profile = base();
        profile.participant_model = ParticipantModel::MultiParty { participants: 3 };
        profile.key_model = FheKeyModel::Threshold { participants: 4, threshold: 2 };
        assert_eq!(profile.validate(), Err(ProfileError::FheKeyParticipantMismatch));
    }

    #[test]
    fn multikey_party_count_must_match_participant_topology() {
        let mut profile = base();
        profile.participant_model = ParticipantModel::MultiParty { participants: 3 };
        profile.key_model = FheKeyModel::MultiKey { participants: 2 };
        assert_eq!(profile.validate(), Err(ProfileError::FheKeyParticipantMismatch));
    }

    #[test]
    fn common_validation_applies_to_fhe() {
        let mut profile = base();
        profile.backend.version = " ".into();
        assert_eq!(profile.validate(), Err(ProfileError::EmptyBackendVersion));
    }

    #[test]
    fn valid_fhe_profile_maps_to_ciphertext_objective() {
        let capability = base().as_capability().unwrap();
        assert_eq!(capability.supported_objectives, vec![PrivacyObjective::ComputeOnCiphertext]);
    }
}
