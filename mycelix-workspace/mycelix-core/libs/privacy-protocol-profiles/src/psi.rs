use crate::{capability, validate_common, ProfileError};
use privacy_computation_core::{
    AdversaryModel, BackendIdentity, InteractionModel, LeakageDeclaration, LeakageProfile,
    ParticipantModel, PrivacyObjective, PrivacyPrimitive, PrimitiveCapability, QualificationState,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PsiOperation {
    Intersection,
    IntersectionCardinality,
    IntersectionAggregate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollectionSemantics {
    Set,
    Multiset,
    OrderedList,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PsiOutputRecipient {
    BothParties,
    ClientOnly,
    ServerOnly,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PsiAggregateFunction {
    Sum,
    Min,
    Max,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PsiAggregateSpec {
    pub function: PsiAggregateFunction,
    /// Versioned identifier for the associated-value interpretation, for
    /// example `u64-cents-v1`. Empty/whitespace-only domains are rejected.
    pub value_domain: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PsiLeakageProfile {
    pub input_sizes: LeakageDeclaration,
    pub result_cardinality: LeakageDeclaration,
    pub result_elements: LeakageDeclaration,
    pub aggregate_value: LeakageDeclaration,
    pub timing: LeakageDeclaration,
    pub message_size: LeakageDeclaration,
    pub abort_behavior: LeakageDeclaration,
    pub cross_session_linkability: LeakageDeclaration,
}

impl Default for PsiLeakageProfile {
    fn default() -> Self {
        Self {
            input_sizes: LeakageDeclaration::Unspecified,
            result_cardinality: LeakageDeclaration::Unspecified,
            result_elements: LeakageDeclaration::Unspecified,
            aggregate_value: LeakageDeclaration::Unspecified,
            timing: LeakageDeclaration::Unspecified,
            message_size: LeakageDeclaration::Unspecified,
            abort_behavior: LeakageDeclaration::Unspecified,
            cross_session_linkability: LeakageDeclaration::Unspecified,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PsiProfile {
    pub backend: BackendIdentity,
    pub participant_model: ParticipantModel,
    pub adversary_model: AdversaryModel,
    pub interaction_model: InteractionModel,
    pub operation: PsiOperation,
    pub collection_semantics: CollectionSemantics,
    pub output_recipient: PsiOutputRecipient,
    pub equality_domain: String,
    pub session_domain: String,
    pub aggregate: Option<PsiAggregateSpec>,
    pub leakage: PsiLeakageProfile,
    pub qualification: QualificationState,
}

impl PsiProfile {
    pub fn validate(&self) -> Result<(), ProfileError> {
        if self.backend.primitive != PrivacyPrimitive::PrivateSetOperation {
            return Err(ProfileError::WrongPrimitive);
        }
        validate_common(&self.backend, self.interaction_model)?;
        if self.participant_model != ParticipantModel::TwoParty {
            return Err(ProfileError::InvalidParticipantModel);
        }
        if self.equality_domain.trim().is_empty() {
            return Err(ProfileError::EmptyPsiEqualityDomain);
        }
        if self.session_domain.trim().is_empty() {
            return Err(ProfileError::EmptyPsiSessionDomain);
        }
        match self.operation {
            PsiOperation::Intersection => {
                if self.aggregate.is_some() {
                    return Err(ProfileError::PsiUnexpectedAggregateSpec);
                }
                if self.leakage.result_elements == LeakageDeclaration::Unspecified {
                    return Err(ProfileError::PsiIntersectionDisclosureUnspecified);
                }
                if self.leakage.result_elements == LeakageDeclaration::MayReveal
                    && self.leakage.result_cardinality == LeakageDeclaration::DeclaredHidden
                {
                    return Err(ProfileError::PsiInconsistentResultLeakage);
                }
            }
            PsiOperation::IntersectionCardinality => {
                if self.aggregate.is_some() {
                    return Err(ProfileError::PsiUnexpectedAggregateSpec);
                }
                if self.leakage.result_elements != LeakageDeclaration::DeclaredHidden {
                    return Err(ProfileError::PsiCardinalityRevealsElements);
                }
                if self.leakage.result_cardinality == LeakageDeclaration::Unspecified {
                    return Err(ProfileError::PsiCardinalityDisclosureUnspecified);
                }
            }
            PsiOperation::IntersectionAggregate => {
                let aggregate = self
                    .aggregate
                    .as_ref()
                    .ok_or(ProfileError::PsiAggregateSpecRequired)?;
                if aggregate.value_domain.trim().is_empty() {
                    return Err(ProfileError::EmptyPsiAggregateValueDomain);
                }
                if self.leakage.result_elements != LeakageDeclaration::DeclaredHidden {
                    return Err(ProfileError::PsiAggregateRevealsElements);
                }
                if self.leakage.aggregate_value == LeakageDeclaration::Unspecified {
                    return Err(ProfileError::PsiAggregateDisclosureUnspecified);
                }
            }
        }
        Ok(())
    }

    pub fn as_capability(&self) -> Result<PrimitiveCapability, ProfileError> {
        self.validate()?;
        let output_value = match self.operation {
            PsiOperation::Intersection => self.leakage.result_elements,
            PsiOperation::IntersectionCardinality => self.leakage.result_cardinality,
            PsiOperation::IntersectionAggregate => self.leakage.aggregate_value,
        };
        Ok(capability(
            self.backend.clone(),
            PrivacyObjective::PrivateSetRelation,
            self.participant_model,
            self.adversary_model,
            self.interaction_model,
            LeakageProfile {
                input_size: self.leakage.input_sizes,
                set_size: self.leakage.input_sizes,
                output_value,
                ..LeakageProfile::default()
            },
            self.qualification,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn backend() -> BackendIdentity {
        BackendIdentity {
            primitive: PrivacyPrimitive::PrivateSetOperation,
            backend: "synthetic".into(),
            version: "0.0.1".into(),
            profile: "structural-test".into(),
        }
    }

    fn base(operation: PsiOperation) -> PsiProfile {
        PsiProfile {
            backend: backend(),
            participant_model: ParticipantModel::TwoParty,
            adversary_model: AdversaryModel::HonestButCurious,
            interaction_model: InteractionModel::Interactive { rounds: 2 },
            operation,
            collection_semantics: CollectionSemantics::Set,
            output_recipient: PsiOutputRecipient::ClientOnly,
            equality_domain: "synthetic-identifiers-v1".into(),
            session_domain: "psi-test-session".into(),
            aggregate: None,
            leakage: PsiLeakageProfile::default(),
            qualification: QualificationState::Experimental,
        }
    }

    fn aggregate_spec() -> PsiAggregateSpec {
        PsiAggregateSpec {
            function: PsiAggregateFunction::Sum,
            value_domain: "u64-cents-v1".into(),
        }
    }

    #[test]
    fn v1_rejects_undefined_multiparty_recipient_semantics() {
        let mut profile = base(PsiOperation::Intersection);
        profile.leakage.result_elements = LeakageDeclaration::MayReveal;
        profile.participant_model = ParticipantModel::MultiParty { participants: 3 };
        assert_eq!(profile.validate(), Err(ProfileError::InvalidParticipantModel));
    }

    #[test]
    fn equality_and_session_domains_are_required() {
        let mut profile = base(PsiOperation::Intersection);
        profile.leakage.result_elements = LeakageDeclaration::MayReveal;
        profile.equality_domain = " ".into();
        assert_eq!(profile.validate(), Err(ProfileError::EmptyPsiEqualityDomain));
        profile.equality_domain = "synthetic-identifiers-v1".into();
        profile.session_domain.clear();
        assert_eq!(profile.validate(), Err(ProfileError::EmptyPsiSessionDomain));
    }

    #[test]
    fn intersection_disclosure_must_be_explicit() {
        let profile = base(PsiOperation::Intersection);
        assert_eq!(
            profile.validate(),
            Err(ProfileError::PsiIntersectionDisclosureUnspecified)
        );
    }

    #[test]
    fn revealed_elements_cannot_claim_hidden_cardinality() {
        let mut profile = base(PsiOperation::Intersection);
        profile.leakage.result_elements = LeakageDeclaration::MayReveal;
        profile.leakage.result_cardinality = LeakageDeclaration::DeclaredHidden;
        assert_eq!(profile.validate(), Err(ProfileError::PsiInconsistentResultLeakage));
    }

    #[test]
    fn cardinality_must_hide_matching_elements() {
        let mut profile = base(PsiOperation::IntersectionCardinality);
        profile.leakage.result_cardinality = LeakageDeclaration::MayReveal;
        profile.leakage.result_elements = LeakageDeclaration::MayReveal;
        assert_eq!(profile.validate(), Err(ProfileError::PsiCardinalityRevealsElements));
    }

    #[test]
    fn cardinality_disclosure_must_be_explicit() {
        let mut profile = base(PsiOperation::IntersectionCardinality);
        profile.leakage.result_elements = LeakageDeclaration::DeclaredHidden;
        assert_eq!(
            profile.validate(),
            Err(ProfileError::PsiCardinalityDisclosureUnspecified)
        );
    }

    #[test]
    fn aggregate_requires_function_and_value_domain() {
        let mut profile = base(PsiOperation::IntersectionAggregate);
        profile.leakage.result_elements = LeakageDeclaration::DeclaredHidden;
        profile.leakage.aggregate_value = LeakageDeclaration::MayReveal;
        assert_eq!(profile.validate(), Err(ProfileError::PsiAggregateSpecRequired));
        profile.aggregate = Some(PsiAggregateSpec {
            function: PsiAggregateFunction::Sum,
            value_domain: " ".into(),
        });
        assert_eq!(profile.validate(), Err(ProfileError::EmptyPsiAggregateValueDomain));
    }

    #[test]
    fn nonaggregate_operation_rejects_aggregate_spec() {
        let mut profile = base(PsiOperation::IntersectionCardinality);
        profile.leakage.result_elements = LeakageDeclaration::DeclaredHidden;
        profile.leakage.result_cardinality = LeakageDeclaration::MayReveal;
        profile.aggregate = Some(aggregate_spec());
        assert_eq!(profile.validate(), Err(ProfileError::PsiUnexpectedAggregateSpec));
    }

    #[test]
    fn aggregate_must_hide_matching_elements_and_bind_aggregate_disclosure() {
        let mut profile = base(PsiOperation::IntersectionAggregate);
        profile.aggregate = Some(aggregate_spec());
        profile.leakage.result_elements = LeakageDeclaration::DeclaredHidden;
        assert_eq!(
            profile.validate(),
            Err(ProfileError::PsiAggregateDisclosureUnspecified)
        );
        profile.leakage.aggregate_value = LeakageDeclaration::MayReveal;
        assert_eq!(profile.validate(), Ok(()));
    }

    #[test]
    fn aggregate_output_never_aliases_cardinality() {
        let mut profile = base(PsiOperation::IntersectionAggregate);
        profile.aggregate = Some(aggregate_spec());
        profile.leakage.result_elements = LeakageDeclaration::DeclaredHidden;
        profile.leakage.result_cardinality = LeakageDeclaration::DeclaredHidden;
        profile.leakage.aggregate_value = LeakageDeclaration::MayReveal;
        let capability = profile.as_capability().unwrap();
        assert_eq!(capability.leakage.output_value, LeakageDeclaration::MayReveal);
    }

    #[test]
    fn valid_cardinality_maps_cardinality_output_only() {
        let mut profile = base(PsiOperation::IntersectionCardinality);
        profile.leakage.result_elements = LeakageDeclaration::DeclaredHidden;
        profile.leakage.result_cardinality = LeakageDeclaration::MayReveal;
        profile.leakage.aggregate_value = LeakageDeclaration::DeclaredHidden;
        let capability = profile.as_capability().unwrap();
        assert_eq!(capability.leakage.output_value, LeakageDeclaration::MayReveal);
        assert_eq!(capability.supported_objectives, vec![PrivacyObjective::PrivateSetRelation]);
    }
}
