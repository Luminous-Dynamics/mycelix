use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::Arc,
};

use axum::{
    body::Body,
    http::{header::CONTENT_LENGTH, Method, Request, StatusCode},
};
use http_body_util::BodyExt as _;
use mycelix_content_core::{BlobDescriptorV1, DigestAlgorithmV1};
use mycelix_content_node::{CasConfigV1, LocalCasV1};
use mycelix_nix_cache::{
    encode_nix_base32, router, NixCacheCatalogV1, NixCacheConfigV1, NixCacheEntryV1,
    NixCacheErrorV1,
};
use tower::ServiceExt as _;

const STORE_HASH: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const OTHER_STORE_HASH: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

fn entry_for(bytes: &[u8]) -> NixCacheEntryV1 {
    let descriptor = BlobDescriptorV1::from_bytes(DigestAlgorithmV1::Sha256, bytes, None);
    NixCacheEntryV1::new(
        &format!("/nix/store/{STORE_HASH}-demo-1.0"),
        descriptor.digest,
        descriptor.size_bytes,
        vec![],
        None,
        vec!["cache.example.org-1:AAAAAAAA=="],
        None,
    )
    .unwrap()
}

fn test_app(bytes: &[u8]) -> (axum::Router, NixCacheEntryV1, tempfile::TempDir) {
    let dir = tempfile::tempdir().unwrap();
    let cas = LocalCasV1::open(CasConfigV1::new(dir.path(), 16 * 1024)).unwrap();
    let descriptor = BlobDescriptorV1::from_bytes(DigestAlgorithmV1::Sha256, bytes, None);
    cas.put(&descriptor, bytes).unwrap();
    let entry = entry_for(bytes);
    let catalog = NixCacheCatalogV1::new(vec![entry.clone()]).unwrap();
    let config = NixCacheConfigV1::loopback(0);
    let app = router(Arc::new(cas), Arc::new(catalog), &config).unwrap();
    (app, entry, dir)
}

#[tokio::test]
async fn nix_cache_info_is_stock_http_cache_metadata() {
    let (app, _, _dir) = test_app(b"raw-nar-fixture");
    let response = app
        .oneshot(
            Request::builder()
                .uri("/nix-cache-info")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let body = response.into_body().collect().await.unwrap().to_bytes();
    assert_eq!(
        &body[..],
        b"StoreDir: /nix/store\nWantMassQuery: 0\nPriority: 40\n"
    );
}

#[tokio::test]
async fn narinfo_is_derived_from_exact_raw_nar_digest_and_preserves_signature() {
    let bytes = b"raw-nar-fixture";
    let (app, entry, _dir) = test_app(bytes);
    let response = app
        .oneshot(
            Request::builder()
                .uri(format!("/{STORE_HASH}.narinfo"))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let body = response.into_body().collect().await.unwrap().to_bytes();
    let text = std::str::from_utf8(&body).unwrap();
    let expected_hash = format!(
        "sha256:{}",
        encode_nix_base32(&entry.nar_digest().bytes)
    );
    assert!(text.contains("Compression: none\n"));
    assert!(text.contains(&format!("FileHash: {expected_hash}\n")));
    assert!(text.contains(&format!("NarHash: {expected_hash}\n")));
    assert!(text.contains("Sig: cache.example.org-1:AAAAAAAA==\n"));
    assert!(text.contains(&format!("URL: {}\n", entry.nar_url())));
}

#[tokio::test]
async fn nar_get_streams_the_same_sha256_verified_cas_bytes() {
    let bytes = b"raw-nar-fixture";
    let (app, entry, _dir) = test_app(bytes);
    let response = app
        .oneshot(
            Request::builder()
                .uri(format!("/{}", entry.nar_url()))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    assert_eq!(
        response.headers()[CONTENT_LENGTH].to_str().unwrap(),
        bytes.len().to_string()
    );
    let body = response.into_body().collect().await.unwrap().to_bytes();
    assert_eq!(&body[..], bytes);
}

#[tokio::test]
async fn nar_head_verifies_but_returns_no_body() {
    let bytes = b"raw-nar-fixture";
    let (app, entry, _dir) = test_app(bytes);
    let response = app
        .oneshot(
            Request::builder()
                .method(Method::HEAD)
                .uri(format!("/{}", entry.nar_url()))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    assert_eq!(
        response.headers()[CONTENT_LENGTH].to_str().unwrap(),
        bytes.len().to_string()
    );
    let body = response.into_body().collect().await.unwrap().to_bytes();
    assert!(body.is_empty());
}

#[tokio::test]
async fn cas_presence_without_catalog_admission_is_not_publication() {
    let dir = tempfile::tempdir().unwrap();
    let cas = LocalCasV1::open(CasConfigV1::new(dir.path(), 16 * 1024)).unwrap();
    let descriptor = BlobDescriptorV1::from_bytes(
        DigestAlgorithmV1::Sha256,
        b"private-or-unpublished-nar",
        None,
    );
    cas.put(&descriptor, b"private-or-unpublished-nar").unwrap();
    let config = NixCacheConfigV1::loopback(0);
    let app = router(
        Arc::new(cas),
        Arc::new(NixCacheCatalogV1::default()),
        &config,
    )
    .unwrap();

    let response = app
        .oneshot(
            Request::builder()
                .uri(format!("/{OTHER_STORE_HASH}.narinfo"))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::NOT_FOUND);
}

#[tokio::test]
async fn admitted_metadata_with_missing_nar_fails_as_unavailable_not_absent() {
    let bytes = b"missing-local-nar";
    let dir = tempfile::tempdir().unwrap();
    let cas = LocalCasV1::open(CasConfigV1::new(dir.path(), 16 * 1024)).unwrap();
    let entry = entry_for(bytes);
    let catalog = NixCacheCatalogV1::new(vec![entry]).unwrap();
    let config = NixCacheConfigV1::loopback(0);
    let app = router(Arc::new(cas), Arc::new(catalog), &config).unwrap();

    let response = app
        .oneshot(
            Request::builder()
                .uri(format!("/{STORE_HASH}.narinfo"))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
}

#[tokio::test]
async fn wrong_nar_hash_url_does_not_alias_catalog_content() {
    let (app, _, _dir) = test_app(b"raw-nar-fixture");
    let wrong_hash = "0".repeat(52);
    let response = app
        .oneshot(
            Request::builder()
                .uri(format!("/nar/{STORE_HASH}-{wrong_hash}.nar"))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::NOT_FOUND);
}

#[tokio::test]
async fn mutation_methods_are_not_registered() {
    let (app, entry, _dir) = test_app(b"raw-nar-fixture");
    let response = app
        .clone()
        .oneshot(
            Request::builder()
                .method(Method::POST)
                .uri(format!("/{STORE_HASH}.narinfo"))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::METHOD_NOT_ALLOWED);

    let response = app
        .oneshot(
            Request::builder()
                .method(Method::PUT)
                .uri(format!("/{}", entry.nar_url()))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::METHOD_NOT_ALLOWED);
}

#[test]
fn public_bind_is_fail_closed() {
    let config = NixCacheConfigV1 {
        bind_addr: SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 8080),
        priority: 40,
        max_concurrent_verifications: 4,
    };
    assert!(matches!(
        config.validate(),
        Err(NixCacheErrorV1::NonLoopbackBind(_))
    ));
}

#[test]
fn raw_nar_publication_rejects_non_sha256_identity() {
    let descriptor = BlobDescriptorV1::from_bytes(
        DigestAlgorithmV1::Blake3_256,
        b"raw-nar-fixture",
        None,
    );
    assert!(matches!(
        NixCacheEntryV1::new(
            &format!("/nix/store/{STORE_HASH}-demo-1.0"),
            descriptor.digest,
            descriptor.size_bytes,
            vec![],
            None,
            vec![],
            None,
        ),
        Err(NixCacheErrorV1::NarDigestMustBeSha256)
    ));
}

#[test]
fn duplicate_store_hash_catalog_entries_fail_closed() {
    let first = entry_for(b"first");
    let second = entry_for(b"second");
    assert!(matches!(
        NixCacheCatalogV1::new(vec![first, second]),
        Err(NixCacheErrorV1::DuplicateStoreHash(_))
    ));
}
