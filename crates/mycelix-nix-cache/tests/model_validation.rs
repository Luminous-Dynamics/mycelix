use mycelix_content_core::{BlobDescriptorV1, DigestAlgorithmV1};
use mycelix_nix_cache::{
    NixCacheEntryV1, NixCacheErrorV1, NixSignatureV1, NixStoreHashV1,
    NixStorePathBaseNameV1,
};

const STORE_HASH: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const REF_HASH: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

fn sha256_descriptor() -> BlobDescriptorV1 {
    BlobDescriptorV1::from_bytes(DigestAlgorithmV1::Sha256, b"nar", None)
}

#[test]
fn store_hash_requires_exact_canonical_nix32() {
    assert!(NixStoreHashV1::parse(STORE_HASH).is_ok());
    assert!(matches!(
        NixStoreHashV1::parse("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"),
        Err(NixCacheErrorV1::InvalidStoreHash(_))
    ));
    assert!(matches!(
        NixStoreHashV1::parse("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"),
        Err(NixCacheErrorV1::InvalidStoreHash(_))
    ));
    assert!(matches!(
        NixStoreHashV1::parse("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"),
        Err(NixCacheErrorV1::InvalidStoreHash(_))
    ));
}

#[test]
fn store_name_rejects_slashes_and_excess_length() {
    assert!(matches!(
        NixStorePathBaseNameV1::parse(&format!("{STORE_HASH}-bad/name")),
        Err(NixCacheErrorV1::InvalidStorePathBaseName(_))
    ));
    let long_name = "x".repeat(212);
    assert!(matches!(
        NixStorePathBaseNameV1::parse(&format!("{STORE_HASH}-{long_name}")),
        Err(NixCacheErrorV1::InvalidStorePathBaseName(_))
    ));
}

#[test]
fn signature_rejects_line_injection_and_malformed_base64_shape() {
    assert!(NixSignatureV1::parse("cache.example-1:AAAA==").is_ok());
    assert!(matches!(
        NixSignatureV1::parse("cache.example-1:AAAA==\nCA: forged"),
        Err(NixCacheErrorV1::InvalidSignature)
    ));
    assert!(matches!(
        NixSignatureV1::parse("cache.example-1:not base64"),
        Err(NixCacheErrorV1::InvalidSignature)
    ));
}

#[test]
fn duplicate_references_are_rejected_not_silently_normalized() {
    let descriptor = sha256_descriptor();
    let reference = format!("{REF_HASH}-dependency-1.0");
    assert!(matches!(
        NixCacheEntryV1::new(
            &format!("/nix/store/{STORE_HASH}-demo-1.0"),
            descriptor.digest,
            descriptor.size_bytes,
            vec![reference.as_str(), reference.as_str()],
            None,
            vec![],
            None,
        ),
        Err(NixCacheErrorV1::DuplicateReference(value)) if value == reference
    ));
}

#[test]
fn signatures_are_deduplicated_without_increasing_apparent_authority() {
    let descriptor = sha256_descriptor();
    let entry = NixCacheEntryV1::new(
        &format!("/nix/store/{STORE_HASH}-demo-1.0"),
        descriptor.digest,
        descriptor.size_bytes,
        vec![],
        None,
        vec!["cache.example-1:AAAA==", "cache.example-1:AAAA=="],
        None,
    )
    .unwrap();
    assert_eq!(entry.signatures().len(), 1);
    assert_eq!(entry.render_narinfo().matches("Sig: ").count(), 1);
}
