use crate::{capability, validate_common, ProfileError};
use privacy_computation_core::{
    AdversaryModel, BackendIdentity, InteractionModel, LeakageDeclaration, LeakageProfile,
    ParticipantModel, PrivacyObjective, PrivacyPrimitive, PrimitiveCapability, QualificationState,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PirServerModel {
    SingleServer,
    MultiServer { servers: u16, max_colluding: u16 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PirQueryShape {
    SingleIndex,
    Batch { max_queries: u16 },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PirProfile {
    pub backend: BackendIdentity,
    pub participant_model: ParticipantModel,
    pub adversary_model: AdversaryModel,
    pub interaction_model: InteractionModel,
    pub server_model: PirServerModel,
    pub query_shape: PirQueryShape,
    pub database_snapshot: String,
    pub leakage: LeakageProfile,
    pub repeated_query_linkability: LeakageDeclaration,
    pub response_size: LeakageDeclaration,
    pub timing: LeakageDeclaration,
    pub qualification: QualificationState,
}

impl PirProfile {
    pub fn validate(&self) -> Result<(), ProfileError> {
        if self.backend.primitive != PrivacyPrimitive::PrivateInformationRetrieval {
            return Err(ProfileError::WrongPrimitive);
        }
        validate_common(&self.backend, self.interaction_model)?;
        if self.database_snapshot.trim().is_empty() {
            return Err(ProfileError::EmptyPirDatabaseSnapshot);
        }
        match (self.server_model, self.participant_model) {
            (PirServerModel::SingleServer, ParticipantModel::TwoParty) => {}
            (
                PirServerModel::MultiServer {
                    servers,
                    max_colluding,
                },
                ParticipantModel::MultiParty { participants },
            ) => {
                if servers < 2 || max_colluding >= servers {
                    return Err(ProfileError::InvalidPirServerModel);
                }
                let expected_participants = servers
                    .checked_add(1)
                    .ok_or(ProfileError::PirParticipantCountOverflow)?;
                if participants != expected_participants {
                    return Err(ProfileError::PirParticipantTopologyMismatch);
                }
            }
            (PirServerModel::MultiServer { servers, max_colluding }, _)
                if servers < 2 || max_colluding >= servers =>
            {
                return Err(ProfileError::InvalidPirServerModel)
            }
            _ => return Err(ProfileError::PirParticipantTopologyMismatch),
        }
        if let PirQueryShape::Batch { max_queries } = self.query_shape {
            if max_queries == 0 {
                return Err(ProfileError::InvalidPirBatchSize);
            }
        }
        if self.leakage.query_index != LeakageDeclaration::DeclaredHidden {
            return Err(ProfileError::PirQueryIndexNotHidden);
        }
        if self.leakage.access_pattern == LeakageDeclaration::DeclaredHidden {
            return Err(ProfileError::PirAccessPatternOverclaim);
        }
        Ok(())
    }

    pub fn as_capability(&self) -> Result<PrimitiveCapability, ProfileError> {
        self.validate()?;
        Ok(capability(
            self.backend.clone(),
            PrivacyObjective::QueryIndexPrivacy,
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
            primitive: PrivacyPrimitive::PrivateInformationRetrieval,
            backend: "synthetic".into(),
            version: "0.0.1".into(),
            profile: "structural-test".into(),
        }
    }

    fn base() -> PirProfile {
        PirProfile {
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
        }
    }

    #[test]
    fn database_snapshot_is_required() {
        let mut profile = base();
        profile.database_snapshot = " ".into();
        assert_eq!(profile.validate(), Err(ProfileError::EmptyPirDatabaseSnapshot));
    }

    #[test]
    fn query_index_hiding_is_required() {
        let mut profile = base();
        profile.leakage.query_index = LeakageDeclaration::MayReveal;
        assert_eq!(profile.validate(), Err(ProfileError::PirQueryIndexNotHidden));
    }

    #[test]
    fn pir_cannot_claim_oram_access_pattern_privacy() {
        let mut profile = base();
        profile.leakage.access_pattern = LeakageDeclaration::DeclaredHidden;
        assert_eq!(profile.validate(), Err(ProfileError::PirAccessPatternOverclaim));
    }

    #[test]
    fn multi_server_collusion_bound_must_be_below_server_count() {
        let mut profile = base();
        profile.server_model = PirServerModel::MultiServer { servers: 2, max_colluding: 2 };
        profile.participant_model = ParticipantModel::MultiParty { participants: 3 };
        assert_eq!(profile.validate(), Err(ProfileError::InvalidPirServerModel));
    }

    #[test]
    fn server_model_and_participant_topology_must_match() {
        let mut profile = base();
        profile.server_model = PirServerModel::MultiServer { servers: 2, max_colluding: 1 };
        assert_eq!(profile.validate(), Err(ProfileError::PirParticipantTopologyMismatch));
        profile.participant_model = ParticipantModel::MultiParty { participants: 3 };
        assert_eq!(profile.validate(), Ok(()));
    }

    #[test]
    fn server_count_overflow_fails_closed() {
        let mut profile = base();
        profile.server_model = PirServerModel::MultiServer {
            servers: u16::MAX,
            max_colluding: 1,
        };
        profile.participant_model = ParticipantModel::MultiParty {
            participants: u16::MAX,
        };
        assert_eq!(profile.validate(), Err(ProfileError::PirParticipantCountOverflow));
    }

    #[test]
    fn batch_size_must_be_nonzero() {
        let mut profile = base();
        profile.query_shape = PirQueryShape::Batch { max_queries: 0 };
        assert_eq!(profile.validate(), Err(ProfileError::InvalidPirBatchSize));
    }

    #[test]
    fn zero_round_interactive_pir_fails_closed() {
        let mut profile = base();
        profile.interaction_model = InteractionModel::Interactive { rounds: 0 };
        assert_eq!(profile.validate(), Err(ProfileError::ZeroRoundInteractive));
    }

    #[test]
    fn valid_pir_maps_to_query_index_objective() {
        let capability = base().as_capability().unwrap();
        assert_eq!(capability.supported_objectives, vec![PrivacyObjective::QueryIndexPrivacy]);
    }

    #[test]
    fn wrong_primitive_fails_closed() {
        let mut profile = base();
        profile.backend.primitive = PrivacyPrimitive::HomomorphicEncryption;
        assert_eq!(profile.validate(), Err(ProfileError::WrongPrimitive));
    }
}
