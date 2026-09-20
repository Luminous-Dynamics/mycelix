use crate::{capability, participant_count, validate_common, ProfileError};
use privacy_computation_core::{
    AdversaryModel, BackendIdentity, InteractionModel, LeakageDeclaration, LeakageProfile,
    ParticipantModel, PrivacyObjective, PrivacyPrimitive, PrimitiveCapability, QualificationState,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MpcComputationClass {
    ArithmeticCircuit,
    BooleanCircuit,
    MixedCircuit,
    SecretSharedAggregation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MpcCorruptionModel {
    Static { max_corrupted: u16 },
    Adaptive { max_corrupted: u16 },
}

impl MpcCorruptionModel {
    fn max_corrupted(self) -> u16 {
        match self {
            Self::Static { max_corrupted } | Self::Adaptive { max_corrupted } => max_corrupted,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MpcOutputPolicy {
    AllParticipants,
    DesignatedParticipants(Vec<u16>),
    NoOutput,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MpcProfile {
    pub backend: BackendIdentity,
    pub participant_model: ParticipantModel,
    pub adversary_model: AdversaryModel,
    pub interaction_model: InteractionModel,
    pub computation_class: MpcComputationClass,
    pub corruption_model: MpcCorruptionModel,
    pub output_policy: MpcOutputPolicy,
    pub leakage: LeakageProfile,
    pub qualification: QualificationState,
}

impl MpcProfile {
    pub fn validate(&self) -> Result<(), ProfileError> {
        if self.backend.primitive != PrivacyPrimitive::MultiPartyComputation {
            return Err(ProfileError::WrongPrimitive);
        }
        validate_common(&self.backend, self.interaction_model)?;
        let participants = participant_count(self.participant_model)
            .filter(|count| *count >= 2)
            .ok_or(ProfileError::InvalidParticipantModel)?;
        if self.corruption_model.max_corrupted() >= participants {
            return Err(ProfileError::InvalidCorruptionThreshold);
        }
        if let MpcOutputPolicy::DesignatedParticipants(recipients) = &self.output_policy {
            if recipients.is_empty() {
                return Err(ProfileError::InvalidDesignatedRecipient);
            }
            let mut seen = BTreeSet::new();
            for recipient in recipients {
                if *recipient == 0 || *recipient > participants {
                    return Err(ProfileError::InvalidDesignatedRecipient);
                }
                if !seen.insert(*recipient) {
                    return Err(ProfileError::DuplicateDesignatedRecipient);
                }
            }
        }
        match &self.output_policy {
            MpcOutputPolicy::NoOutput => {
                if self.leakage.output_value == LeakageDeclaration::MayReveal {
                    return Err(ProfileError::MpcNoOutputDisclosureConflict);
                }
            }
            MpcOutputPolicy::AllParticipants | MpcOutputPolicy::DesignatedParticipants(_) => {
                if self.leakage.output_value == LeakageDeclaration::Unspecified {
                    return Err(ProfileError::MpcOutputDisclosureUnspecified);
                }
            }
        }
        Ok(())
    }

    pub fn as_capability(&self) -> Result<PrimitiveCapability, ProfileError> {
        self.validate()?;
        Ok(capability(
            self.backend.clone(),
            PrivacyObjective::JointInputConfidentiality,
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
            primitive: PrivacyPrimitive::MultiPartyComputation,
            backend: "synthetic".into(),
            version: "0.0.1".into(),
            profile: "structural-test".into(),
        }
    }

    fn base() -> MpcProfile {
        MpcProfile {
            backend: backend(),
            participant_model: ParticipantModel::MultiParty { participants: 3 },
            adversary_model: AdversaryModel::Malicious,
            interaction_model: InteractionModel::Interactive { rounds: 3 },
            computation_class: MpcComputationClass::ArithmeticCircuit,
            corruption_model: MpcCorruptionModel::Static { max_corrupted: 1 },
            output_policy: MpcOutputPolicy::AllParticipants,
            leakage: LeakageProfile {
                output_value: LeakageDeclaration::MayReveal,
                ..LeakageProfile::default()
            },
            qualification: QualificationState::Experimental,
        }
    }

    #[test]
    fn corruption_bound_must_be_below_party_count() {
        let mut profile = base();
        profile.corruption_model = MpcCorruptionModel::Static { max_corrupted: 3 };
        assert_eq!(profile.validate(), Err(ProfileError::InvalidCorruptionThreshold));
    }

    #[test]
    fn designated_recipients_must_be_unique_and_in_range() {
        let mut profile = base();
        profile.output_policy = MpcOutputPolicy::DesignatedParticipants(vec![1, 1]);
        assert_eq!(profile.validate(), Err(ProfileError::DuplicateDesignatedRecipient));
        profile.output_policy = MpcOutputPolicy::DesignatedParticipants(vec![1, 4]);
        assert_eq!(profile.validate(), Err(ProfileError::InvalidDesignatedRecipient));
    }

    #[test]
    fn output_producing_mpc_requires_explicit_output_disclosure() {
        let mut profile = base();
        profile.leakage.output_value = LeakageDeclaration::Unspecified;
        assert_eq!(profile.validate(), Err(ProfileError::MpcOutputDisclosureUnspecified));
    }

    #[test]
    fn no_output_cannot_claim_output_may_reveal() {
        let mut profile = base();
        profile.output_policy = MpcOutputPolicy::NoOutput;
        assert_eq!(profile.validate(), Err(ProfileError::MpcNoOutputDisclosureConflict));
        profile.leakage.output_value = LeakageDeclaration::DeclaredHidden;
        assert_eq!(profile.validate(), Ok(()));
    }

    #[test]
    fn common_validation_applies_to_mpc() {
        let mut profile = base();
        profile.backend.profile.clear();
        assert_eq!(profile.validate(), Err(ProfileError::EmptyBackendProfile));
    }

    #[test]
    fn valid_mpc_profile_maps_structurally_only() {
        let capability = base().as_capability().unwrap();
        assert_eq!(capability.supported_objectives, vec![PrivacyObjective::JointInputConfidentiality]);
    }
}
