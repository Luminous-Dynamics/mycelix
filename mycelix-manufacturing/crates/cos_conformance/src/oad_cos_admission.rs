//! Runtime-neutral OAD -> COS admission reference model.
use crate::seam_profile::{admit_after_receipt, validate_envelope, Receipt, ReceiptStage, SeamEnvelope, SeamProfile, SemanticDecision};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceStatus { EpisodeDescription, DevelopmentGuideProposal, TechnicalSpecification, RatifiedSchema, Implementation, ConformanceEvidence }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdmissionDecision { Admitted, RejectedStale, RejectedCertificationOnly, RejectedUnauthorized, Indeterminate }

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OadDesignPackage {
    pub design_id: String,
    pub design_generation: u32,
    pub source_status: SourceStatus,
    pub source_schema_version: String,
    pub certified: bool,
    pub production_profile_id: String,
    pub required_material_generation: String,
    pub required_skill_generation: String,
    pub lifecycle_model_generation: String,
    pub ecological_model_generation: String,
    pub superseded: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CosProductionBasis {
    pub design_id: String,
    pub design_generation: u32,
    pub production_profile_id: String,
    pub accepted_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProductionAuthorization {
    pub authorization_id: String,
    pub design_id: String,
    pub production_profile_id: String,
    pub authorized_at: u64,
}

pub fn admit_design_to_cos(design: &OadDesignPackage, expected_profile: &str, expected_generation: u32) -> AdmissionDecision {
    if design.superseded || design.production_profile_id != expected_profile || design.design_generation != expected_generation {
        return AdmissionDecision::RejectedStale;
    }
    if !design.certified { return AdmissionDecision::RejectedCertificationOnly; }
    AdmissionDecision::Admitted
}

pub fn certification_grants_production_authority(_design: &OadDesignPackage) -> bool { false }

pub fn create_production_basis(design: &OadDesignPackage, expected_profile: &str, expected_generation: u32, accepted_at: u64) -> Option<CosProductionBasis> {
    match admit_design_to_cos(design, expected_profile, expected_generation) {
        AdmissionDecision::Admitted => Some(CosProductionBasis {
            design_id: design.design_id.clone(),
            design_generation: design.design_generation,
            production_profile_id: design.production_profile_id.clone(),
            accepted_at,
        }),
        _ => None,
    }
}

pub fn authorization_is_explicit(authorization: Option<&ProductionAuthorization>, basis: &CosProductionBasis) -> bool {
    authorization.is_some_and(|a| a.design_id == basis.design_id && a.production_profile_id == basis.production_profile_id)
}

pub fn seam_to_cos_admission(profile: &SeamProfile, envelope: &SeamEnvelope, receipt: &Receipt) -> AdmissionDecision {
    if validate_envelope(profile, envelope) != SemanticDecision::Accepted { return AdmissionDecision::RejectedStale; }
    match admit_after_receipt(profile, envelope, receipt) {
        SemanticDecision::Accepted if receipt.stage == ReceiptStage::RecipientSemanticallyAdmitted => AdmissionDecision::Admitted,
        SemanticDecision::Rejected | SemanticDecision::StaleSchema => AdmissionDecision::RejectedStale,
        SemanticDecision::Indeterminate => AdmissionDecision::Indeterminate,
        SemanticDecision::PayloadMismatch | SemanticDecision::Unauthorized => AdmissionDecision::RejectedUnauthorized,
        SemanticDecision::Accepted => AdmissionDecision::Admitted,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seam_profile::{DeliveryMode, ReceiptStage, SemanticDecision};

    fn design() -> OadDesignPackage {
        OadDesignPackage {
            design_id: "oad-greenhouse-001".into(), design_generation: 7,
            source_status: SourceStatus::EpisodeDescription,
            source_schema_version: "oad.public.2026-09".into(), certified: true,
            production_profile_id: "node-h2-profile-1".into(),
            required_material_generation: "materials-3".into(),
            required_skill_generation: "skills-4".into(),
            lifecycle_model_generation: "lifecycle-2".into(),
            ecological_model_generation: "ecology-5".into(), superseded: false,
        }
    }

    fn profile() -> SeamProfile {
        SeamProfile {
            profile_id: "INTEGRAL-OAD-COS-001".into(), profile_version: 1,
            sender_domain: "OAD".into(), receiver_domain: "COS".into(),
            source_schema_version: "oad.public.2026-09".into(),
            delivery_mode: DeliveryMode::Event,
            retry_idempotency_profile: "stable-delivery-id".into(), ordering_guaranteed: false,
        }
    }

    fn envelope() -> SeamEnvelope {
        SeamEnvelope {
            profile_id: "INTEGRAL-OAD-COS-001".into(), profile_version: 1,
            semantic_subject_id: "oad-greenhouse-001:g7".into(),
            payload_commitment: "sha256:design".into(),
            delivery_id: "delivery-oad-1".into(), attempt_id: "attempt-1".into(),
            source_schema_version: "oad.public.2026-09".into(), origin: "node-a".into(),
            authority_reference: None,
        }
    }

    #[test] fn certified_design_is_not_production_authority() {
        assert!(!certification_grants_production_authority(&design()));
    }

    #[test] fn stale_or_wrong_generation_cannot_be_current_production_basis() {
        let d = design();
        assert_eq!(admit_design_to_cos(&d, "node-h2-profile-1", 6), AdmissionDecision::RejectedStale);
        assert_eq!(admit_design_to_cos(&d, "different-profile", 7), AdmissionDecision::RejectedStale);
    }

    #[test] fn uncertified_design_is_not_admitted() {
        let mut d = design(); d.certified = false;
        assert_eq!(admit_design_to_cos(&d, "node-h2-profile-1", 7), AdmissionDecision::RejectedCertificationOnly);
    }

    #[test] fn admission_creates_basis_but_not_authorization() {
        let basis = create_production_basis(&design(), "node-h2-profile-1", 7, 100).unwrap();
        assert!(!authorization_is_explicit(None, &basis));
    }

    #[test] fn explicit_authorization_must_match_basis() {
        let basis = create_production_basis(&design(), "node-h2-profile-1", 7, 100).unwrap();
        let wrong = ProductionAuthorization { authorization_id: "auth-1".into(), design_id: basis.design_id.clone(), production_profile_id: "other-profile".into(), authorized_at: 101 };
        assert!(!authorization_is_explicit(Some(&wrong), &basis));
        let right = ProductionAuthorization { authorization_id: "auth-2".into(), design_id: basis.design_id.clone(), production_profile_id: basis.production_profile_id.clone(), authorized_at: 101 };
        assert!(authorization_is_explicit(Some(&right), &basis));
    }

    #[test] fn transport_acceptance_does_not_admit_oad_design_to_cos() {
        let p = profile(); let e = envelope();
        let r = Receipt { delivery_id: e.delivery_id.clone(), attempt_id: e.attempt_id.clone(), stage: ReceiptStage::TransportAccepted, semantic_decision: None };
        assert_eq!(seam_to_cos_admission(&p, &e, &r), AdmissionDecision::Indeterminate);
    }

    #[test] fn semantic_admission_is_required_before_cos_can_accept_design_reference() {
        let p = profile(); let e = envelope();
        let r = Receipt { delivery_id: e.delivery_id.clone(), attempt_id: e.attempt_id.clone(), stage: ReceiptStage::RecipientSemanticallyAdmitted, semantic_decision: Some(SemanticDecision::Accepted) };
        assert_eq!(seam_to_cos_admission(&p, &e, &r), AdmissionDecision::Admitted);
    }

    #[test] fn foreign_origin_survives_admission() {
        let p = profile(); let mut e = envelope(); e.origin = "node-b".into();
        let r = Receipt { delivery_id: e.delivery_id.clone(), attempt_id: e.attempt_id.clone(), stage: ReceiptStage::RecipientSemanticallyAdmitted, semantic_decision: Some(SemanticDecision::Accepted) };
        assert_eq!(seam_to_cos_admission(&p, &e, &r), AdmissionDecision::Admitted);
        assert_eq!(e.origin, "node-b");
    }
}
