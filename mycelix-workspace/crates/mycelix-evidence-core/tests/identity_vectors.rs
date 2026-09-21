use mycelix_evidence_core::{ArtifactId, ClaimId, EVIDENCE_IDENTITY_PROFILE_V1};

#[test]
fn artifact_vector_is_stable() {
    let id = ArtifactId::new("web", "example.org/report:v1").unwrap();
    assert_eq!(EVIDENCE_IDENTITY_PROFILE_V1, "mycelix:epistemic-identity:v1");
    assert_eq!(
        id.commitment_sha256_hex(),
        "bde8c523cf8daa4201a28c49a84f09830579b0072e78a892ecc7f78976d727f7"
    );
}

#[test]
fn same_strings_in_another_role_are_a_different_identity() {
    let artifact = ArtifactId::new("web", "example.org/report:v1").unwrap();
    let claim = ClaimId::new("web", "example.org/report:v1").unwrap();
    assert_ne!(artifact.commitment_sha256_hex(), claim.commitment_sha256_hex());
    assert_eq!(
        claim.commitment_sha256_hex(),
        "fc8b830da362c54f7fd05f8457c85a74187970890d558ba9c2743d4f7f371683"
    );
}
