use mycelix_content_core::*;
use mycelix_infrastructure_types::{PartyIdV1, UnixMillisV1};

fn media(value: &str) -> MediaTypeV1 {
    MediaTypeV1::new(value).unwrap()
}

#[test]
fn digest_algorithm_is_part_of_identity() {
    let bytes = b"same bytes";
    let blake = ContentDigestV1::compute(DigestAlgorithmV1::Blake3_256, bytes);
    let sha = ContentDigestV1::compute(DigestAlgorithmV1::Sha256, bytes);
    assert_ne!(blake.algorithm, sha.algorithm);
    assert_ne!(blake.bytes, sha.bytes);
}

#[test]
fn manifest_order_is_identity_bearing() {
    let a = BlobDescriptorV1::from_bytes(
        DigestAlgorithmV1::Blake3_256,
        b"a",
        Some(media("application/octet-stream")),
    );
    let b = BlobDescriptorV1::from_bytes(
        DigestAlgorithmV1::Blake3_256,
        b"b",
        Some(media("application/octet-stream")),
    );
    let left = ObjectManifestV1::new(vec![a.clone(), b.clone()], None).unwrap();
    let right = ObjectManifestV1::new(vec![b, a], None).unwrap();
    assert_ne!(left.id, right.id);
}

#[test]
fn publication_provenance_does_not_change_object_identity() {
    let blob = BlobDescriptorV1::from_bytes(DigestAlgorithmV1::Blake3_256, b"shared", None);
    let manifest = ObjectManifestV1::new(vec![blob], None).unwrap();
    let p1 = PublicationRecordV1::new(
        manifest.id,
        Some(LogicalNameV1::new("shared-object").unwrap()),
        Some(VersionLabelV1::new("v1").unwrap()),
        PartyIdV1([1; 32]),
        UnixMillisV1(100),
        None,
    )
    .unwrap();
    let p2 = PublicationRecordV1::new(
        manifest.id,
        Some(LogicalNameV1::new("shared-object").unwrap()),
        Some(VersionLabelV1::new("v1").unwrap()),
        PartyIdV1([2; 32]),
        UnixMillisV1(100),
        None,
    )
    .unwrap();
    assert_eq!(p1.object_id, p2.object_id);
    assert_ne!(p1.id, p2.id);
}

#[test]
fn manifest_rejects_tampered_total() {
    let blob = BlobDescriptorV1::from_bytes(DigestAlgorithmV1::Blake3_256, b"abc", None);
    let mut manifest = ObjectManifestV1::new(vec![blob], None).unwrap();
    manifest.total_size_bytes += 1;
    assert_eq!(
        manifest.validate().unwrap_err(),
        ContentErrorV1::TotalSizeMismatch,
    );
}

#[test]
fn manifest_requires_at_least_one_blob() {
    assert_eq!(
        ObjectManifestV1::new(Vec::new(), None).unwrap_err(),
        ContentErrorV1::EmptyManifest,
    );
}
