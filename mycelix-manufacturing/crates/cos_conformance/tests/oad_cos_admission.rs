use cos_conformance::oad_cos_admission::{
    admit_design_to_cos, authorization_is_explicit, certification_grants_production_authority,
    create_production_basis, AdmissionDecision, OadDesignPackage, ProductionAuthorization, SourceStatus,
};

fn design() -> OadDesignPackage {
    OadDesignPackage {
        design_id: "design-1".into(), design_generation: 3,
        source_status: SourceStatus::RatifiedSchema,
        source_schema_version: "oad.public.2026-09".into(), certified: true,
        production_profile_id: "profile-1".into(),
        required_material_generation: "materials-1".into(),
        required_skill_generation: "skills-1".into(),
        lifecycle_model_generation: "lifecycle-1".into(),
        ecological_model_generation: "ecology-1".into(), superseded: false,
    }
}

#[test]
fn current_certified_design_can_be_admitted_without_becoming_authority() {
    let d = design();
    assert_eq!(admit_design_to_cos(&d, "profile-1", 3), AdmissionDecision::Admitted);
    assert!(!certification_grants_production_authority(&d));
}

#[test]
fn stale_design_is_rejected_even_if_certified() {
    assert_eq!(admit_design_to_cos(&design(), "profile-1", 2), AdmissionDecision::RejectedStale);
}

#[test]
fn wrong_profile_is_rejected_even_if_generation_matches() {
    assert_eq!(admit_design_to_cos(&design(), "profile-2", 3), AdmissionDecision::RejectedStale);
}

#[test]
fn basis_is_not_authorization() {
    let basis = create_production_basis(&design(), "profile-1", 3, 100).unwrap();
    assert!(!authorization_is_explicit(None, &basis));
}

#[test]
fn matching_authorization_is_explicit() {
    let basis = create_production_basis(&design(), "profile-1", 3, 100).unwrap();
    let authorization = ProductionAuthorization {
        authorization_id: "auth-1".into(),
        design_id: basis.design_id.clone(),
        production_profile_id: basis.production_profile_id.clone(),
        authorized_at: 101,
    };
    assert!(authorization_is_explicit(Some(&authorization), &basis));
}

#[test]
fn non_ratified_source_status_cannot_establish_cos_admission_or_basis() {
    for status in [
        SourceStatus::EpisodeDescription,
        SourceStatus::DevelopmentGuideProposal,
        SourceStatus::TechnicalSpecification,
        SourceStatus::Implementation,
        SourceStatus::ConformanceEvidence,
    ] {
        let mut candidate = design();
        candidate.source_status = status;
        assert_eq!(
            admit_design_to_cos(&candidate, "profile-1", 3),
            AdmissionDecision::RejectedSourceStatus,
            "source status {status:?} must not establish a production basis"
        );
        assert!(create_production_basis(&candidate, "profile-1", 3, 100).is_none());
    }
}

#[test]
fn ratified_source_status_is_still_not_production_authority() {
    let candidate = design();
    assert_eq!(
        admit_design_to_cos(&candidate, "profile-1", 3),
        AdmissionDecision::Admitted
    );
    let basis = create_production_basis(&candidate, "profile-1", 3, 100).unwrap();
    assert!(!authorization_is_explicit(None, &basis));
    assert!(!certification_grants_production_authority(&candidate));
}
