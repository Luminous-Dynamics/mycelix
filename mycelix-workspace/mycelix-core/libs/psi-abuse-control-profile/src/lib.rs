// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B: authority-neutral abuse-control composition profiles for PSI-002A.
//!
//! This crate deliberately does not implement authentication, rate limiting,
//! storage, networking, or raw-identifier inspection. It describes what a
//! surrounding admission service must bind before any enumeration-resistance
//! experiment may be claimed.
//!
//! ```text
//! bounded budget != enumeration resistance
//! scoped capability != Sybil resistance
//! blinded request != anonymous client
//! ```

use privacy_computation_core::{LeakageDeclaration, ParticipantModel, QualificationState};
use privacy_protocol_profiles::{
    CollectionSemantics, PsiOperation, PsiOutputRecipient, PsiProfile,
};
use serde::{Deserialize, Serialize};

pub const PSI_002A_SUBJECT: &str = "2be72da2acfd9903bfca168035c9ee087059f46f";
pub const PSI_002A_MAX_IDENTIFIERS_PER_REQUEST: u32 = 1_024;
pub const PROFILE_ID: &str = "psi-abuse-control-v1";
pub const PSI_BACKEND: &str = "facebook/voprf";
pub const PSI_BACKEND_VERSION: &str = "0.5.0";
pub const PSI_BACKEND_PROFILE: &str = "voprf-tagged-set-v1/ristretto255-SHA512/f0531f081238";
pub const MAX_IDENTITY_BYTES: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RawIdentifierPolicy {
    /// Admission receives no raw queried identifiers. It may account for only
    /// blinded request metadata and externally supplied policy facts.
    ForbiddenAtAdmissionBoundary,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RequestCommitmentPolicy {
    /// Every admitted request must carry a commitment that an external durable
    /// store proves has not already been consumed in the capability epoch.
    UniquePerCapabilityEpoch,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RevocationPolicy {
    /// Capability revocation state must be checked before every admission.
    CheckBeforeEveryAdmission,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QueryBudget {
    pub max_requests_per_epoch: u32,
    pub max_blinded_elements_per_request: u32,
    pub max_blinded_elements_per_epoch: u64,
    pub max_concurrent_requests: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdmissionMetadataLeakage {
    pub capability_identity: LeakageDeclaration,
    pub request_count: LeakageDeclaration,
    pub blinded_element_count: LeakageDeclaration,
    pub timing: LeakageDeclaration,
    pub network_identity: LeakageDeclaration,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PsiAbuseControlProfile {
    pub profile_id: String,
    pub psi_subject: String,
    pub service_domain: String,
    pub epoch_domain: String,
    pub psi_equality_domain: String,
    pub psi_session_domain: String,
    pub capability_namespace: String,
    pub budget: QueryBudget,
    pub raw_identifier_policy: RawIdentifierPolicy,
    pub request_commitment_policy: RequestCommitmentPolicy,
    pub revocation_policy: RevocationPolicy,
    pub metadata_leakage: AdmissionMetadataLeakage,
    pub qualification: QualificationState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProfileError {
    EmptyProfileId,
    WrongProfileId,
    WrongPsiSubject,
    InvalidServiceDomain,
    InvalidEpochDomain,
    InvalidCapabilityNamespace,
    ZeroRequestBudget,
    ZeroRequestElementBudget,
    ZeroEpochElementBudget,
    ZeroConcurrencyBudget,
    RequestElementBudgetExceedsPsiCeiling,
    EpochElementBudgetSmallerThanRequestBudget,
    EpochElementBudgetExceedsRequestEnvelope,
    ConcurrencyExceedsRequestBudget,
    MetadataLeakageUnspecified,
    CapabilityIdentityMustRevealToAdmissionService,
    RequestCountMustRevealToAdmissionService,
    BlindedElementCountMustRevealToAdmissionService,
    TimingMustRevealToAdmissionService,
    QualificationMustRemainExperimental,
    PsiProfileInvalid,
    WrongPsiBackend,
    WrongPsiBackendVersion,
    WrongPsiBackendProfile,
    WrongPsiEqualityDomain,
    WrongPsiSessionDomain,
    WrongPsiOperation,
    WrongCollectionSemantics,
    WrongOutputRecipient,
    WrongParticipantModel,
    WrongPsiQualification,
}

fn valid_identity(value: &str) -> bool {
    let value = value.as_bytes();
    !value.is_empty()
        && value.len() <= MAX_IDENTITY_BYTES
        && value.iter().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(*byte, b'-' | b'_' | b'.' | b':' | b'/')
        })
}

impl QueryBudget {
    pub fn validate(&self) -> Result<(), ProfileError> {
        if self.max_requests_per_epoch == 0 {
            return Err(ProfileError::ZeroRequestBudget);
        }
        if self.max_blinded_elements_per_request == 0 {
            return Err(ProfileError::ZeroRequestElementBudget);
        }
        if self.max_blinded_elements_per_epoch == 0 {
            return Err(ProfileError::ZeroEpochElementBudget);
        }
        if self.max_concurrent_requests == 0 {
            return Err(ProfileError::ZeroConcurrencyBudget);
        }
        if self.max_blinded_elements_per_request > PSI_002A_MAX_IDENTIFIERS_PER_REQUEST {
            return Err(ProfileError::RequestElementBudgetExceedsPsiCeiling);
        }
        if self.max_blinded_elements_per_epoch < u64::from(self.max_blinded_elements_per_request) {
            return Err(ProfileError::EpochElementBudgetSmallerThanRequestBudget);
        }
        let envelope = u64::from(self.max_requests_per_epoch)
            .checked_mul(u64::from(self.max_blinded_elements_per_request))
            .ok_or(ProfileError::EpochElementBudgetExceedsRequestEnvelope)?;
        if self.max_blinded_elements_per_epoch > envelope {
            return Err(ProfileError::EpochElementBudgetExceedsRequestEnvelope);
        }
        if u32::from(self.max_concurrent_requests) > self.max_requests_per_epoch {
            return Err(ProfileError::ConcurrencyExceedsRequestBudget);
        }
        Ok(())
    }
}

impl AdmissionMetadataLeakage {
    pub fn validate(&self) -> Result<(), ProfileError> {
        let dimensions = [
            self.capability_identity,
            self.request_count,
            self.blinded_element_count,
            self.timing,
            self.network_identity,
        ];
        if dimensions.contains(&LeakageDeclaration::Unspecified) {
            return Err(ProfileError::MetadataLeakageUnspecified);
        }
        if self.capability_identity != LeakageDeclaration::MayReveal {
            return Err(ProfileError::CapabilityIdentityMustRevealToAdmissionService);
        }
        if self.request_count != LeakageDeclaration::MayReveal {
            return Err(ProfileError::RequestCountMustRevealToAdmissionService);
        }
        if self.blinded_element_count != LeakageDeclaration::MayReveal {
            return Err(ProfileError::BlindedElementCountMustRevealToAdmissionService);
        }
        if self.timing != LeakageDeclaration::MayReveal {
            return Err(ProfileError::TimingMustRevealToAdmissionService);
        }
        Ok(())
    }
}

impl PsiAbuseControlProfile {
    pub fn validate(&self) -> Result<(), ProfileError> {
        if self.profile_id.trim().is_empty() {
            return Err(ProfileError::EmptyProfileId);
        }
        if self.profile_id != PROFILE_ID {
            return Err(ProfileError::WrongProfileId);
        }
        if self.psi_subject != PSI_002A_SUBJECT {
            return Err(ProfileError::WrongPsiSubject);
        }
        if !valid_identity(&self.service_domain) {
            return Err(ProfileError::InvalidServiceDomain);
        }
        if !valid_identity(&self.epoch_domain) {
            return Err(ProfileError::InvalidEpochDomain);
        }
        if !valid_identity(&self.psi_equality_domain) {
            return Err(ProfileError::WrongPsiEqualityDomain);
        }
        if !valid_identity(&self.psi_session_domain) {
            return Err(ProfileError::WrongPsiSessionDomain);
        }
        if !valid_identity(&self.capability_namespace) {
            return Err(ProfileError::InvalidCapabilityNamespace);
        }
        self.budget.validate()?;
        self.metadata_leakage.validate()?;
        if self.qualification != QualificationState::Experimental {
            return Err(ProfileError::QualificationMustRemainExperimental);
        }
        Ok(())
    }

    /// Structurally binds this admission profile to the exact PSI shape used by
    /// PSI-002A. This is compatibility evidence only.
    pub fn validate_psi_binding(&self, psi: &PsiProfile) -> Result<(), ProfileError> {
        self.validate()?;
        psi.validate().map_err(|_| ProfileError::PsiProfileInvalid)?;
        if psi.backend.backend != PSI_BACKEND {
            return Err(ProfileError::WrongPsiBackend);
        }
        if psi.backend.version != PSI_BACKEND_VERSION {
            return Err(ProfileError::WrongPsiBackendVersion);
        }
        if psi.backend.profile != PSI_BACKEND_PROFILE {
            return Err(ProfileError::WrongPsiBackendProfile);
        }
        if psi.equality_domain != self.psi_equality_domain {
            return Err(ProfileError::WrongPsiEqualityDomain);
        }
        if psi.session_domain != self.psi_session_domain {
            return Err(ProfileError::WrongPsiSessionDomain);
        }
        if psi.operation != PsiOperation::Intersection {
            return Err(ProfileError::WrongPsiOperation);
        }
        if psi.collection_semantics != CollectionSemantics::Set {
            return Err(ProfileError::WrongCollectionSemantics);
        }
        if psi.output_recipient != PsiOutputRecipient::ClientOnly {
            return Err(ProfileError::WrongOutputRecipient);
        }
        if psi.participant_model != ParticipantModel::TwoParty {
            return Err(ProfileError::WrongParticipantModel);
        }
        if psi.qualification != QualificationState::Experimental {
            return Err(ProfileError::WrongPsiQualification);
        }
        Ok(())
    }

    pub const fn raw_identifiers_accepted_by_admission(&self) -> bool {
        false
    }

    pub const fn unique_raw_guess_enforcement_available(&self) -> bool {
        false
    }

    pub const fn online_enumeration_resistance_established(&self) -> bool {
        false
    }

    pub const fn sybil_resistance_established(&self) -> bool {
        false
    }

    pub const fn client_anonymity_established(&self) -> bool {
        false
    }

    pub const fn privacy_preserving_accounting_established(&self) -> bool {
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
    use privacy_computation_core::{
        AdversaryModel, BackendIdentity, InteractionModel, PrivacyPrimitive,
    };
    use privacy_protocol_profiles::PsiLeakageProfile;

    fn profile() -> PsiAbuseControlProfile {
        PsiAbuseControlProfile {
            profile_id: PROFILE_ID.into(),
            psi_subject: PSI_002A_SUBJECT.into(),
            service_domain: "contact-discovery-v1".into(),
            epoch_domain: "epoch-0001".into(),
            psi_equality_domain: "mycelix-test:contact-discovery-v1".into(),
            psi_session_domain: "session-a".into(),
            capability_namespace: "contact-discovery-query-v1".into(),
            budget: QueryBudget {
                max_requests_per_epoch: 16,
                max_blinded_elements_per_request: 64,
                max_blinded_elements_per_epoch: 512,
                max_concurrent_requests: 2,
            },
            raw_identifier_policy: RawIdentifierPolicy::ForbiddenAtAdmissionBoundary,
            request_commitment_policy: RequestCommitmentPolicy::UniquePerCapabilityEpoch,
            revocation_policy: RevocationPolicy::CheckBeforeEveryAdmission,
            metadata_leakage: AdmissionMetadataLeakage {
                capability_identity: LeakageDeclaration::MayReveal,
                request_count: LeakageDeclaration::MayReveal,
                blinded_element_count: LeakageDeclaration::MayReveal,
                timing: LeakageDeclaration::MayReveal,
                network_identity: LeakageDeclaration::MayReveal,
            },
            qualification: QualificationState::Experimental,
        }
    }

    fn psi() -> PsiProfile {
        PsiProfile {
            backend: BackendIdentity {
                primitive: PrivacyPrimitive::PrivateSetOperation,
                backend: "facebook/voprf".into(),
                version: "0.5.0".into(),
                profile: "voprf-tagged-set-v1/ristretto255-SHA512/f0531f081238".into(),
            },
            participant_model: ParticipantModel::TwoParty,
            adversary_model: AdversaryModel::HonestButCurious,
            interaction_model: InteractionModel::Interactive { rounds: 2 },
            operation: PsiOperation::Intersection,
            collection_semantics: CollectionSemantics::Set,
            output_recipient: PsiOutputRecipient::ClientOnly,
            equality_domain: "mycelix-test:contact-discovery-v1".into(),
            session_domain: "session-a".into(),
            aggregate: None,
            leakage: PsiLeakageProfile {
                input_sizes: LeakageDeclaration::MayReveal,
                result_cardinality: LeakageDeclaration::MayReveal,
                result_elements: LeakageDeclaration::MayReveal,
                aggregate_value: LeakageDeclaration::Unspecified,
                timing: LeakageDeclaration::MayReveal,
                message_size: LeakageDeclaration::MayReveal,
                abort_behavior: LeakageDeclaration::MayReveal,
                cross_session_linkability: LeakageDeclaration::MayReveal,
            },
            qualification: QualificationState::Experimental,
        }
    }

    #[test]
    fn valid_profile_binds_exact_psi_shape() {
        let profile = profile();
        assert_eq!(profile.validate(), Ok(()));
        assert_eq!(profile.validate_psi_binding(&psi()), Ok(()));
    }

    #[test]
    fn exact_psi_subject_is_required() {
        let mut profile = profile();
        profile.psi_subject = "other".into();
        assert_eq!(profile.validate(), Err(ProfileError::WrongPsiSubject));
    }

    #[test]
    fn zero_request_budget_rejected() {
        let mut profile = profile();
        profile.budget.max_requests_per_epoch = 0;
        assert_eq!(profile.validate(), Err(ProfileError::ZeroRequestBudget));
    }

    #[test]
    fn request_batch_cannot_exceed_psi_product_ceiling() {
        let mut profile = profile();
        profile.budget.max_blinded_elements_per_request = PSI_002A_MAX_IDENTIFIERS_PER_REQUEST + 1;
        assert_eq!(profile.validate(), Err(ProfileError::RequestElementBudgetExceedsPsiCeiling));
    }

    #[test]
    fn epoch_budget_must_cover_one_allowed_request() {
        let mut profile = profile();
        profile.budget.max_blinded_elements_per_epoch = 63;
        assert_eq!(profile.validate(), Err(ProfileError::EpochElementBudgetSmallerThanRequestBudget));
    }

    #[test]
    fn epoch_budget_cannot_exceed_request_envelope() {
        let mut profile = profile();
        profile.budget.max_blinded_elements_per_epoch = 2_000;
        assert_eq!(profile.validate(), Err(ProfileError::EpochElementBudgetExceedsRequestEnvelope));
    }

    #[test]
    fn concurrency_cannot_exceed_request_budget() {
        let mut profile = profile();
        profile.budget.max_requests_per_epoch = 1;
        profile.budget.max_concurrent_requests = 2;
        assert_eq!(profile.validate(), Err(ProfileError::ConcurrencyExceedsRequestBudget));
    }

    #[test]
    fn server_visible_accounting_must_not_pretend_capability_is_hidden() {
        let mut profile = profile();
        profile.metadata_leakage.capability_identity = LeakageDeclaration::DeclaredHidden;
        assert_eq!(profile.validate(), Err(ProfileError::CapabilityIdentityMustRevealToAdmissionService));
    }

    #[test]
    fn metadata_leakage_must_be_explicit() {
        let mut profile = profile();
        profile.metadata_leakage.timing = LeakageDeclaration::Unspecified;
        assert_eq!(profile.validate(), Err(ProfileError::MetadataLeakageUnspecified));
    }

    #[test]
    fn qualification_cannot_self_promote() {
        let mut profile = profile();
        profile.qualification = QualificationState::Qualified;
        assert_eq!(profile.validate(), Err(ProfileError::QualificationMustRemainExperimental));
    }

    #[test]
    fn exact_backend_profile_is_required() {
        let profile = profile();
        let mut psi = psi();
        psi.backend.profile = "other".into();
        assert_eq!(profile.validate_psi_binding(&psi), Err(ProfileError::WrongPsiBackendProfile));
    }

    #[test]
    fn exact_psi_domains_are_required() {
        let profile = profile();
        let mut psi = psi();
        psi.session_domain = "session-b".into();
        assert_eq!(profile.validate_psi_binding(&psi), Err(ProfileError::WrongPsiSessionDomain));
    }

    #[test]
    fn admission_timing_cannot_be_declared_hidden() {
        let mut profile = profile();
        profile.metadata_leakage.timing = LeakageDeclaration::DeclaredHidden;
        assert_eq!(profile.validate(), Err(ProfileError::TimingMustRevealToAdmissionService));
    }

    #[test]
    fn wrong_psi_operation_rejected() {
        let profile = profile();
        let mut psi = psi();
        psi.operation = PsiOperation::IntersectionCardinality;
        psi.leakage.result_elements = LeakageDeclaration::DeclaredHidden;
        assert_eq!(profile.validate_psi_binding(&psi), Err(ProfileError::WrongPsiOperation));
    }

    #[test]
    fn authority_ceiling_never_promotes_abuse_resistance() {
        let profile = profile();
        assert!(!profile.raw_identifiers_accepted_by_admission());
        assert!(!profile.unique_raw_guess_enforcement_available());
        assert!(!profile.online_enumeration_resistance_established());
        assert!(!profile.sybil_resistance_established());
        assert!(!profile.client_anonymity_established());
        assert!(!profile.privacy_preserving_accounting_established());
        assert!(!profile.production_admission_granted());
        assert!(!profile.application_authority_granted());
    }
}
