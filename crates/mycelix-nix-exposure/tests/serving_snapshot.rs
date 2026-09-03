use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1};
use mycelix_infrastructure_types::PartyIdV1;
use mycelix_nix_cache::{NixCacheCatalogV1, NixCacheEntryV1, NixStoreHashV1};
use mycelix_nix_exposure::{
    project_remote_serving_snapshot_v1, ExposureAssuranceV1, ExposureAudienceV1,
    ExposureEndpointIdV1, ExposureErrorV1, ExposureEvidenceCoverageV1,
    RemoteExposureGrantEvidenceV1, RemoteExposurePolicyV1, RemoteReaderV1,
    RemoteServingProjectionErrorV1, RemoteServingSnapshotErrorV1,
};

const STORE_HASH: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const EVALUATION: u64 = 2_000;
const REFRESH: u64 = 1_000;

fn authority() -> PartyIdV1 {
    PartyIdV1([1; 32])
}

fn fixture() -> (
    NixCacheCatalogV1,
    ExposureEndpointIdV1,
    RemoteExposureGrantEvidenceV1,
) {
    let bytes = b"serving-safe-snapshot";
    let digest = ContentDigestV1::compute(DigestAlgorithmV1::Sha256, bytes);
    let entry = NixCacheEntryV1::new(
        &format!("/nix/store/{STORE_HASH}-serving-safe"),
        digest,
        bytes.len() as u64,
        vec![],
        None,
        vec!["cache.example-1:AAAA=="],
        None,
    )
    .unwrap();
    let endpoint = ExposureEndpointIdV1::derive(authority(), "edge-jhb-1").unwrap();
    let grant = RemoteExposureGrantEvidenceV1::for_entry(
        authority(),
        endpoint,
        &entry,
        ExposureAudienceV1::Public,
        1_000,
        1_000,
        5_000,
        ExposureAssuranceV1::CryptographicallyVerified,
    )
    .unwrap();
    (NixCacheCatalogV1::new(vec![entry]).unwrap(), endpoint, grant)
}

#[test]
fn strict_partial_coverage_cannot_cross_serving_boundary() {
    let (catalog, endpoint, grant) = fixture();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 10_000, 10_000).unwrap();
    let error = project_remote_serving_snapshot_v1(
        &catalog,
        &[grant],
        &[],
        EVALUATION,
        ExposureEvidenceCoverageV1::Partial,
        &policy,
        REFRESH,
    )
    .unwrap_err();

    assert!(matches!(
        error,
        RemoteServingProjectionErrorV1::Projection(ExposureErrorV1::IncompleteCoverage)
    ));
}

#[test]
fn policy_that_allows_incomplete_coverage_is_not_serving_safe() {
    let (catalog, endpoint, grant) = fixture();
    let policy = RemoteExposurePolicyV1::new(
        endpoint,
        ExposureAssuranceV1::CryptographicallyVerified,
        10_000,
        10_000,
        false,
        true,
        false,
        false,
    )
    .unwrap();
    let error = project_remote_serving_snapshot_v1(
        &catalog,
        &[grant],
        &[],
        EVALUATION,
        ExposureEvidenceCoverageV1::CompleteForAuthority,
        &policy,
        REFRESH,
    )
    .unwrap_err();

    assert!(matches!(
        error,
        RemoteServingProjectionErrorV1::Promotion(
            RemoteServingSnapshotErrorV1::PolicyAllowsIncompleteCoverage
        )
    ));
}

#[test]
fn operator_only_policy_is_not_promoted_to_strict_serving_snapshot() {
    let (catalog, endpoint, grant) = fixture();
    let policy = RemoteExposurePolicyV1::new(
        endpoint,
        ExposureAssuranceV1::OperatorVerified,
        10_000,
        10_000,
        true,
        true,
        false,
        false,
    )
    .unwrap();
    let error = project_remote_serving_snapshot_v1(
        &catalog,
        &[grant],
        &[],
        EVALUATION,
        ExposureEvidenceCoverageV1::CompleteForAuthority,
        &policy,
        REFRESH,
    )
    .unwrap_err();

    assert!(matches!(
        error,
        RemoteServingProjectionErrorV1::Promotion(
            RemoteServingSnapshotErrorV1::AssuranceTooLow
        )
    ));
}

#[test]
fn zero_refresh_interval_is_rejected() {
    let (catalog, endpoint, grant) = fixture();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 10_000, 10_000).unwrap();
    let error = project_remote_serving_snapshot_v1(
        &catalog,
        &[grant],
        &[],
        EVALUATION,
        ExposureEvidenceCoverageV1::CompleteForAuthority,
        &policy,
        0,
    )
    .unwrap_err();

    assert!(matches!(
        error,
        RemoteServingProjectionErrorV1::Promotion(
            RemoteServingSnapshotErrorV1::ZeroRefreshInterval
        )
    ));
}

#[test]
fn strict_complete_evidence_produces_time_bounded_reader_scoped_serving_snapshot() {
    let (catalog, endpoint, grant) = fixture();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 10_000, 10_000).unwrap();
    let serving = project_remote_serving_snapshot_v1(
        &catalog,
        &[grant],
        &[],
        EVALUATION,
        ExposureEvidenceCoverageV1::CompleteForAuthority,
        &policy,
        REFRESH,
    )
    .unwrap();
    let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();

    assert_eq!(serving.len(), 1);
    assert_eq!(serving.authority(), authority());
    assert_ne!(serving.id(), serving.projection_id());
    assert_eq!(serving.serve_until_unix_ms(), 3_000);
    assert!(serving.is_valid_at(EVALUATION));
    assert!(serving.is_valid_at(2_999));
    assert!(!serving.is_valid_at(3_000));
    assert!(
        serving
            .entry_for_reader_at(&hash, &RemoteReaderV1::anonymous(), 2_999)
            .unwrap()
            .is_some()
    );
    assert_eq!(
        serving
            .entry_for_reader_at(&hash, &RemoteReaderV1::anonymous(), 3_000)
            .unwrap_err(),
        RemoteServingSnapshotErrorV1::SnapshotNotValidAt {
            now_unix_ms: 3_000,
            evaluation_time_unix_ms: EVALUATION,
            serve_until_unix_ms: 3_000,
        }
    );
}
