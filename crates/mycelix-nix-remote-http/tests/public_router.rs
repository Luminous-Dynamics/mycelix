use std::{
    sync::{
        atomic::{AtomicU64, Ordering},
        Arc,
    },
};

use axum::{
    body::Body,
    http::{
        header::{CACHE_CONTROL, CONTENT_LENGTH, RETRY_AFTER},
        Request, StatusCode,
    },
};
use http_body_util::BodyExt;
use mycelix_content_core::{BlobDescriptorV1, DigestAlgorithmV1};
use mycelix_content_node::{CasConfigV1, LocalCasV1};
use mycelix_infrastructure_types::PartyIdV1;
use mycelix_nix_cache::{NixCacheCatalogV1, NixCacheEntryV1};
use mycelix_nix_exposure::{
    project_remote_serving_snapshot_v1, ExposureAssuranceV1, ExposureAudienceV1,
    ExposureEndpointIdV1, ExposureEvidenceCoverageV1, RemoteExposureGrantEvidenceV1,
    RemoteExposurePolicyV1, RemoteServingSnapshotV1,
};
use mycelix_nix_remote_http::{
    public_router, RemoteClockErrorV1, RemoteClockV1, RemoteNixRouterConfigV1,
    RemoteNixRouterErrorV1,
};
use tempfile::TempDir;
use tower::ServiceExt;

const STORE_HASH: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const EVALUATION: u64 = 2_000;
const DEADLINE: u64 = 3_000;

#[derive(Debug)]
struct MutableClock {
    now: AtomicU64,
}

impl MutableClock {
    fn new(now: u64) -> Self {
        Self {
            now: AtomicU64::new(now),
        }
    }

    fn set(&self, now: u64) {
        self.now.store(now, Ordering::SeqCst);
    }
}

impl RemoteClockV1 for MutableClock {
    fn now_unix_ms(&self) -> Result<u64, RemoteClockErrorV1> {
        Ok(self.now.load(Ordering::SeqCst))
    }
}

#[derive(Debug)]
struct FailingClock;

impl RemoteClockV1 for FailingClock {
    fn now_unix_ms(&self) -> Result<u64, RemoteClockErrorV1> {
        Err(RemoteClockErrorV1::BeforeUnixEpoch)
    }
}

#[derive(Debug)]
struct AdvancingClock {
    calls: AtomicU64,
}

impl AdvancingClock {
    fn new() -> Self {
        Self {
            calls: AtomicU64::new(0),
        }
    }
}

impl RemoteClockV1 for AdvancingClock {
    fn now_unix_ms(&self) -> Result<u64, RemoteClockErrorV1> {
        let call = self.calls.fetch_add(1, Ordering::SeqCst);
        Ok(if call == 0 { EVALUATION } else { DEADLINE })
    }
}

struct Fixture {
    _dir: TempDir,
    app: axum::Router,
    clock: Arc<MutableClock>,
    cas: Arc<LocalCasV1>,
    snapshot: Arc<RemoteServingSnapshotV1>,
    entry: NixCacheEntryV1,
    nar: Vec<u8>,
}

fn authority() -> PartyIdV1 {
    PartyIdV1([1; 32])
}

fn principal() -> PartyIdV1 {
    PartyIdV1([2; 32])
}

fn fixture(audience: ExposureAudienceV1) -> Fixture {
    let nar = b"remote-public-nar".to_vec();
    let descriptor = BlobDescriptorV1::from_bytes(DigestAlgorithmV1::Sha256, &nar, None);
    let dir = tempfile::tempdir().unwrap();
    let cas = Arc::new(LocalCasV1::open(CasConfigV1::new(dir.path(), 1024 * 1024)).unwrap());
    cas.put(&descriptor, nar.as_slice()).unwrap();

    let entry = NixCacheEntryV1::new(
        &format!("/nix/store/{STORE_HASH}-remote-demo"),
        descriptor.digest,
        descriptor.size_bytes,
        vec![],
        None,
        vec!["cache.example-1:AAAA=="],
        None,
    )
    .unwrap();
    let catalog = NixCacheCatalogV1::new(vec![entry.clone()]).unwrap();
    let endpoint = ExposureEndpointIdV1::derive(authority(), "edge-jhb-1").unwrap();
    let grant = RemoteExposureGrantEvidenceV1::for_entry(
        authority(),
        endpoint,
        &entry,
        audience,
        1_000,
        1_000,
        5_000,
        ExposureAssuranceV1::CryptographicallyVerified,
    )
    .unwrap();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 10_000, 10_000).unwrap();
    let snapshot = Arc::new(
        project_remote_serving_snapshot_v1(
            &catalog,
            &[grant],
            &[],
            EVALUATION,
            ExposureEvidenceCoverageV1::CompleteForAuthority,
            &policy,
            DEADLINE - EVALUATION,
        )
        .unwrap(),
    );
    let clock = Arc::new(MutableClock::new(EVALUATION));
    let app = public_router(
        cas.clone(),
        snapshot.clone(),
        clock.clone(),
        RemoteNixRouterConfigV1::default(),
    )
    .unwrap();

    Fixture {
        _dir: dir,
        app,
        clock,
        cas,
        snapshot,
        entry,
        nar,
    }
}

async fn body_bytes(response: axum::response::Response) -> Vec<u8> {
    response
        .into_body()
        .collect()
        .await
        .unwrap()
        .to_bytes()
        .to_vec()
}

#[tokio::test]
async fn public_entry_serves_stock_nix_metadata_and_verified_nar_before_deadline() {
    let fixture = fixture(ExposureAudienceV1::Public);

    let info = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .uri("/nix-cache-info")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(info.status(), StatusCode::OK);
    assert_eq!(info.headers()[CACHE_CONTROL], "no-store");

    let narinfo = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .uri(format!("/{STORE_HASH}.narinfo"))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(narinfo.status(), StatusCode::OK);
    assert_eq!(narinfo.headers()[CACHE_CONTROL], "no-store");
    let text = String::from_utf8(body_bytes(narinfo).await).unwrap();
    assert!(text.contains(fixture.entry.store_path().as_str()));
    assert!(text.contains("Compression: none"));

    let nar_response = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .uri(format!("/{}", fixture.entry.nar_url()))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(nar_response.status(), StatusCode::OK);
    assert_eq!(nar_response.headers()[CACHE_CONTROL], "no-store");
    assert_eq!(body_bytes(nar_response).await, fixture.nar);
}

#[tokio::test]
async fn head_preserves_representation_length_without_sending_body() {
    let fixture = fixture(ExposureAudienceV1::Public);
    let expected_narinfo_len = fixture.entry.render_narinfo().len().to_string();
    let expected_nar_len = fixture.nar.len().to_string();

    let narinfo = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .method("HEAD")
                .uri(format!("/{STORE_HASH}.narinfo"))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(narinfo.status(), StatusCode::OK);
    assert_eq!(narinfo.headers()[CACHE_CONTROL], "no-store");
    assert_eq!(narinfo.headers()[CONTENT_LENGTH], expected_narinfo_len.as_str());
    assert!(body_bytes(narinfo).await.is_empty());

    let nar = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .method("HEAD")
                .uri(format!("/{}", fixture.entry.nar_url()))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(nar.status(), StatusCode::OK);
    assert_eq!(nar.headers()[CACHE_CONTROL], "no-store");
    assert_eq!(nar.headers()[CONTENT_LENGTH], expected_nar_len.as_str());
    assert!(body_bytes(nar).await.is_empty());
}

#[tokio::test]
async fn expired_serving_snapshot_fails_closed_before_content_lookup() {
    let fixture = fixture(ExposureAudienceV1::Public);
    fixture.clock.set(DEADLINE);

    for uri in [
        "/nix-cache-info".to_string(),
        format!("/{STORE_HASH}.narinfo"),
        format!("/{}", fixture.entry.nar_url()),
    ] {
        let response = fixture
            .app
            .clone()
            .oneshot(Request::builder().uri(uri).body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
        assert_eq!(response.headers()[RETRY_AFTER], "1");
        assert_eq!(response.headers()[CACHE_CONTROL], "no-store");
    }
}

#[tokio::test]
async fn clock_rollback_cannot_resurrect_an_expired_serving_snapshot() {
    let fixture = fixture(ExposureAudienceV1::Public);
    fixture.clock.set(DEADLINE);
    let expired = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .uri("/nix-cache-info")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(expired.status(), StatusCode::SERVICE_UNAVAILABLE);

    fixture.clock.set(DEADLINE - 1);
    let rolled_back = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .uri("/nix-cache-info")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(rolled_back.status(), StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(rolled_back.headers()[RETRY_AFTER], "1");
    assert_eq!(rolled_back.headers()[CACHE_CONTROL], "no-store");
}

#[tokio::test]
async fn restricted_existing_entry_and_absent_entry_are_both_not_found() {
    let fixture = fixture(ExposureAudienceV1::AuthenticatedPrincipal(principal()));

    let restricted = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .uri(format!("/{STORE_HASH}.narinfo"))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    let absent = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .uri("/bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.narinfo")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(restricted.status(), StatusCode::NOT_FOUND);
    assert_eq!(absent.status(), StatusCode::NOT_FOUND);
    assert_eq!(restricted.headers()[CACHE_CONTROL], "no-store");
    assert_eq!(absent.headers()[CACHE_CONTROL], "no-store");
}

#[tokio::test]
async fn authorized_entry_with_missing_local_nar_is_service_unavailable() {
    let fixture = fixture(ExposureAudienceV1::Public);
    let empty_dir = tempfile::tempdir().unwrap();
    let empty_cas = Arc::new(
        LocalCasV1::open(CasConfigV1::new(empty_dir.path(), 1024 * 1024)).unwrap(),
    );
    let app = public_router(
        empty_cas,
        fixture.snapshot.clone(),
        Arc::new(MutableClock::new(EVALUATION)),
        RemoteNixRouterConfigV1::default(),
    )
    .unwrap();

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
    assert_eq!(response.headers()[RETRY_AFTER], "1");
    assert_eq!(response.headers()[CACHE_CONTROL], "no-store");
}

#[tokio::test]
async fn wrong_nar_hash_does_not_reveal_authorized_content() {
    let fixture = fixture(ExposureAudienceV1::Public);
    let response = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .uri(format!(
                    "/nar/{STORE_HASH}-0000000000000000000000000000000000000000000000000000.nar"
                ))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::NOT_FOUND);
    assert_eq!(response.headers()[CACHE_CONTROL], "no-store");
}

#[tokio::test]
async fn mutation_methods_and_unmatched_paths_are_no_store() {
    let fixture = fixture(ExposureAudienceV1::Public);
    let method = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .method("PUT")
                .uri(format!("/{STORE_HASH}.narinfo"))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(method.status(), StatusCode::METHOD_NOT_ALLOWED);
    assert_eq!(method.headers()[CACHE_CONTROL], "no-store");

    let missing = fixture
        .app
        .clone()
        .oneshot(
            Request::builder()
                .uri("/not/a/nix/cache/route")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(missing.status(), StatusCode::NOT_FOUND);
    assert_eq!(missing.headers()[CACHE_CONTROL], "no-store");
}

#[tokio::test]
async fn clock_failure_fails_closed_as_authority_unavailable() {
    let fixture = fixture(ExposureAudienceV1::Public);
    let app = public_router(
        fixture.cas.clone(),
        fixture.snapshot.clone(),
        Arc::new(FailingClock),
        RemoteNixRouterConfigV1::default(),
    )
    .unwrap();

    let response = app
        .oneshot(
            Request::builder()
                .uri("/nix-cache-info")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(response.headers()[RETRY_AFTER], "1");
    assert_eq!(response.headers()[CACHE_CONTROL], "no-store");
}

#[tokio::test]
async fn authority_is_rechecked_after_full_verification_before_response_commit() {
    let fixture = fixture(ExposureAudienceV1::Public);
    let app = public_router(
        fixture.cas.clone(),
        fixture.snapshot.clone(),
        Arc::new(AdvancingClock::new()),
        RemoteNixRouterConfigV1::default(),
    )
    .unwrap();

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
    assert_eq!(response.headers()[RETRY_AFTER], "1");
    assert_eq!(response.headers()[CACHE_CONTROL], "no-store");
}

#[test]
fn verification_concurrency_is_fail_closed() {
    assert!(matches!(
        RemoteNixRouterConfigV1 {
            priority: 40,
            max_concurrent_verifications: 0,
        }
        .validate(),
        Err(RemoteNixRouterErrorV1::ZeroVerificationConcurrency)
    ));
    assert!(matches!(
        RemoteNixRouterConfigV1 {
            priority: 40,
            max_concurrent_verifications: 65,
        }
        .validate(),
        Err(RemoteNixRouterErrorV1::VerificationConcurrencyTooHigh { .. })
    ));
}
