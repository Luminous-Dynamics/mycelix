use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1};
use mycelix_infrastructure_types::PartyIdV1;
use mycelix_nix_cache::{NixCacheCatalogV1, NixCacheEntryV1};
use mycelix_nix_exposure::{
    project_remote_serving_snapshot_v1, ExposureAssuranceV1, ExposureAudienceV1,
    ExposureEndpointIdV1, ExposureEvidenceCoverageV1, RemoteExposureGrantEvidenceV1,
    RemoteExposurePolicyV1, RemoteExposureRevocationEvidenceV1, RemoteServingSnapshotV1,
};

const STORE_HASH: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const EVALUATION: u64 = 2_000;

fn authority() -> PartyIdV1 {
    PartyIdV1([1; 32])
}

fn entry() -> NixCacheEntryV1 {
    let bytes = b"serving-horizon";
    let digest = ContentDigestV1::compute(DigestAlgorithmV1::Sha256, bytes);
    NixCacheEntryV1::new(
        &format!("/nix/store/{STORE_HASH}-serving-horizon"),
        digest,
        bytes.len() as u64,
        vec![],
        None,
        vec!["cache.example-1:AAAA=="],
        None,
    )
    .unwrap()
}

fn grant(
    entry: &NixCacheEntryV1,
    endpoint: ExposureEndpointIdV1,
    valid_until: u64,
) -> RemoteExposureGrantEvidenceV1 {
    RemoteExposureGrantEvidenceV1::for_entry(
        authority(),
        endpoint,
        entry,
        ExposureAudienceV1::Public,
        1_000,
        1_000,
        valid_until,
        ExposureAssuranceV1::CryptographicallyVerified,
    )
    .unwrap()
}

fn serving(
    entry: NixCacheEntryV1,
    grant: &RemoteExposureGrantEvidenceV1,
    revocations: &[RemoteExposureRevocationEvidenceV1],
    policy: &RemoteExposurePolicyV1,
    refresh_interval_ms: u64,
) -> RemoteServingSnapshotV1 {
    project_remote_serving_snapshot_v1(
        &NixCacheCatalogV1::new(vec![entry]).unwrap(),
        std::slice::from_ref(grant),
        revocations,
        EVALUATION,
        ExposureEvidenceCoverageV1::CompleteForAuthority,
        policy,
        refresh_interval_ms,
    )
    .unwrap()
}

#[test]
fn refresh_interval_can_bound_serving_horizon() {
    let entry = entry();
    let endpoint = ExposureEndpointIdV1::derive(authority(), "edge-jhb-1").unwrap();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 20_000, 20_000).unwrap();
    let grant = grant(&entry, endpoint, 10_000);
    let serving = serving(entry, &grant, &[], &policy, 700);
    assert_eq!(serving.serve_until_unix_ms(), 2_700);
}

#[test]
fn grant_expiry_clamps_serving_horizon() {
    let entry = entry();
    let endpoint = ExposureEndpointIdV1::derive(authority(), "edge-jhb-1").unwrap();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 20_000, 20_000).unwrap();
    let grant = grant(&entry, endpoint, 3_500);
    let serving = serving(entry, &grant, &[], &policy, 10_000);
    assert_eq!(serving.serve_until_unix_ms(), 3_500);
}

#[test]
fn evidence_freshness_clamps_serving_horizon() {
    let entry = entry();
    let endpoint = ExposureEndpointIdV1::derive(authority(), "edge-jhb-1").unwrap();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 20_000, 1_500).unwrap();
    let grant = grant(&entry, endpoint, 10_000);
    let serving = serving(entry, &grant, &[], &policy, 10_000);
    assert_eq!(serving.serve_until_unix_ms(), 2_500);
}

#[test]
fn trusted_scheduled_revocation_clamps_serving_horizon_from_same_evidence_set() {
    let entry = entry();
    let endpoint = ExposureEndpointIdV1::derive(authority(), "edge-jhb-1").unwrap();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 20_000, 20_000).unwrap();
    let grant = grant(&entry, endpoint, 10_000);
    let revocation = RemoteExposureRevocationEvidenceV1::new(
        grant.id(),
        authority(),
        1_500,
        3_000,
        ExposureAssuranceV1::CryptographicallyVerified,
    )
    .unwrap();
    let serving = serving(entry, &grant, std::slice::from_ref(&revocation), &policy, 10_000);
    assert_eq!(serving.serve_until_unix_ms(), 3_000);
}

#[test]
fn below_policy_scheduled_revocation_does_not_shorten_horizon() {
    let entry = entry();
    let endpoint = ExposureEndpointIdV1::derive(authority(), "edge-jhb-1").unwrap();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 20_000, 20_000).unwrap();
    let grant = grant(&entry, endpoint, 5_000);
    let weak = RemoteExposureRevocationEvidenceV1::new(
        grant.id(),
        authority(),
        1_500,
        2_500,
        ExposureAssuranceV1::SelfClaimed,
    )
    .unwrap();
    let serving = serving(entry, &grant, std::slice::from_ref(&weak), &policy, 10_000);
    assert_eq!(serving.serve_until_unix_ms(), 5_000);
}

#[test]
fn predating_revocation_ignored_by_projection_does_not_shorten_horizon() {
    let entry = entry();
    let endpoint = ExposureEndpointIdV1::derive(authority(), "edge-jhb-1").unwrap();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 20_000, 20_000).unwrap();
    let grant = grant(&entry, endpoint, 5_000);
    let predating = RemoteExposureRevocationEvidenceV1::new(
        grant.id(),
        authority(),
        900,
        2_500,
        ExposureAssuranceV1::CryptographicallyVerified,
    )
    .unwrap();
    let serving = serving(entry, &grant, std::slice::from_ref(&predating), &policy, 10_000);
    assert_eq!(serving.serve_until_unix_ms(), 5_000);
}

#[test]
fn serving_identity_commits_refresh_configuration_and_deadline() {
    let entry = entry();
    let endpoint = ExposureEndpointIdV1::derive(authority(), "edge-jhb-1").unwrap();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 20_000, 20_000).unwrap();
    let grant = grant(&entry, endpoint, 10_000);
    let a = serving(entry.clone(), &grant, &[], &policy, 500);
    let b = serving(entry, &grant, &[], &policy, 800);

    assert_ne!(a.id(), b.id());
    assert_ne!(a.serve_until_unix_ms(), b.serve_until_unix_ms());
}
