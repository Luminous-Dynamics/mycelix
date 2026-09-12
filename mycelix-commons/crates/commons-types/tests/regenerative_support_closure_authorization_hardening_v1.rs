include!("regenerative_support_closure_continuity_fixture.rs");

#[test]
fn surface_authorization_rejects_a_substituted_safe_parent_basis() {
    let viability = viability(false);
    let surface = surface(&viability);
    let basis = basis(&viability, Some(&surface));
    let continuity = direct_continuity(&viability, &basis, &surface);

    let mut substituted_basis = basis.clone();
    substituted_basis.basis_evidence_id = "manta-v3-v4-topology-basis-substitute".into();
    assert!(substituted_basis.validate().is_ok());
    assert_ne!(
        substituted_basis.content_digest().unwrap(),
        continuity.support_basis_evidence_content_digest
    );

    assert!(verify_regenerative_support_closure_surface_authorization(
        &substituted_basis,
        &continuity,
        &surface,
    )
    .is_err());
}

#[test]
fn hidden_support_subject_cannot_reuse_direct_viability_evidence() {
    let direct_viability = viability(false);
    let direct_surface = surface(&direct_viability);
    let direct_basis = basis(&direct_viability, Some(&direct_surface));
    let hidden = hidden_continuity(&direct_viability, &direct_basis);

    assert!(verify_regenerative_support_closure_continuity_evidence(
        &direct_viability,
        &direct_basis,
        &hidden,
    )
    .is_err());
}
