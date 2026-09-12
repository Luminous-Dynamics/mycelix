include!("regenerative_support_closure_continuity_fixture.rs");

#[test]
fn surface_authorization_rejects_a_substituted_safe_parent_basis() {
    let viability = viability();
    let surface = surface(&viability);
    let basis = basis(&viability, &surface);
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
