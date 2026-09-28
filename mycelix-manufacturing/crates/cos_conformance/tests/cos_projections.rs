use cos_conformance::cos_projections::{
    project_to_frs, project_to_itc, projection_grants_authority, CosObservation, ObservationKind,
};
use cos_conformance::Evidence;

fn labor() -> CosObservation {
    CosObservation {
        id: "labor-1".into(),
        kind: ObservationKind::Labor,
        source_evidence: Evidence::current_local("evidence-1", 100),
        observed_at: 100,
        verified: true,
        participant_ref: Some("participant-1".into()),
        labor_hours: Some(4),
        material_ref: None,
    }
}

#[test]
fn itc_projection_keeps_source_observation_identity_and_values() {
    let projection = project_to_itc(&labor(), 1).unwrap();
    assert_eq!(projection.source_observation_id, "labor-1");
    assert_eq!(projection.labor_hours, Some(4));
    assert!(!projection_grants_authority());
}

#[test]
fn frs_signal_is_derived_from_source_observation() {
    let signal = project_to_frs(&labor(), 1).unwrap();
    assert_eq!(signal.source_observation_ids, vec!["labor-1"]);
    assert!(signal.interpretation.contains("not a replacement"));
}
