use mycelix_evidence_core::{ArtifactId, ClaimId, EVIDENCE_IDENTITY_PROFILE_V1};

#[test]
fn artifact_vector_is_stable() {
    let id = ArtifactId::new("web", "example.org/report:v1").unwrap();
    assert_eq!(EVIDENCE_IDENTITY_PROFILE_V1, "mycelix:epistemic-identity:v1");
    assert_eq!(
        id.commitment_sha256_hex(),
        "f202795b18c8847ddbc0d232d21b947e338d444c58d8763ddae0db84ae469f17"
    );
}

#[test]
fn same_strings_in_another_role_are_a_different_identity() {
    let artifact = ArtifactId::new("web", "example.org/report:v1").unwrap();
    let claim = ClaimId::new("web", "example.org/report:v1").unwrap();
    assert_ne!(artifact.commitment_sha256_hex(), claim.commitment_sha256_hex());
    assert_eq!(
        claim.commitment_sha256_hex(),
        "40b6c31e6a2398534155f1091b872359318289378f58f014013db769daefb24d"
    );
}
