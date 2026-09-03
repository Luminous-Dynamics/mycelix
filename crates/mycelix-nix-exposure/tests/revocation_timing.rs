use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1};
use mycelix_infrastructure_types::PartyIdV1;
use mycelix_nix_cache::{NixCacheCatalogV1, NixCacheEntryV1};
use mycelix_nix_exposure::{
    project_remote_exposure_v1, ExposureAssuranceV1, ExposureAudienceV1,
    ExposureEndpointIdV1, ExposureEvidenceCoverageV1, RemoteExposureGrantEvidenceV1,
    RemoteExposurePolicyV1, RemoteExposureRevocationEvidenceV1,
};

const STORE_HASH: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

fn authority() -> PartyIdV1 {
    PartyIdV1([1; 32])
}

fn entry() -> NixCacheEntryV1 {
    let bytes = b"scheduled-revocation-nar";
    let digest = ContentDigestV1::compute(DigestAlgorithmV1::Sha256, bytes);
    NixCacheEntryV1::new(
        &format!("/nix/store/{STORE_HASH}-scheduled-revocation"),
        digest,
        bytes.len() as u64,
        vec![],
        None,
        vec!["cache.example-1:AAAA=="],
        None,
    )
    .unwrap()
}

#[test]
fn authored_future_effective_revocation_activates_only_at_effective_boundary() {
    let entry = entry();
    let endpoint = ExposureEndpointIdV1::derive(authority(), "edge-jhb-1").unwrap();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 10_000, 10_000).unwrap();
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
    let revocation = RemoteExposureRevocationEvidenceV1::new(
        grant.id(),
        authority(),
        1_500,
        3_000,
        ExposureAssuranceV1::CryptographicallyVerified,
    )
    .unwrap();
    let catalog = NixCacheCatalogV1::new(vec![entry]).unwrap();

    let before_without = project_remote_exposure_v1(
        &catalog,
        &[grant.clone()],
        &[],
        2_000,
        ExposureEvidenceCoverageV1::CompleteForAuthority,
        &policy,
    )
    .unwrap();
    let before_with = project_remote_exposure_v1(
        &catalog,
        &[grant.clone()],
        &[revocation.clone()],
        2_000,
        ExposureEvidenceCoverageV1::CompleteForAuthority,
        &policy,
    )
    .unwrap();
    assert_eq!(before_without, before_with);
    assert_eq!(before_with.len(), 1);

    let at_effective = project_remote_exposure_v1(
        &catalog,
        &[grant],
        &[revocation],
        3_000,
        ExposureEvidenceCoverageV1::CompleteForAuthority,
        &policy,
    )
    .unwrap();
    assert!(at_effective.is_empty());
}
