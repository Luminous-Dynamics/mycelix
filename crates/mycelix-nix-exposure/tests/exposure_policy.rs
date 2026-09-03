use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1};
use mycelix_infrastructure_types::{PartyIdV1, StableIdV1};
use mycelix_nix_cache::{NixCacheCatalogV1, NixCacheEntryV1, NixStoreHashV1};
use mycelix_nix_exposure::{
    project_remote_exposure_v1, ExposureAssuranceV1, ExposureAudienceV1,
    ExposureEndpointIdV1, ExposureErrorV1, ExposureEvidenceCoverageV1,
    GrantExclusionReasonV1, NixExposureObjectIdV1, RemoteExposureDiagnosticV1,
    RemoteExposureGrantEvidenceV1, RemoteExposurePolicyV1,
    RemoteExposureRevocationEvidenceV1, RemoteReaderV1,
};

const STORE_HASH: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const OTHER_HASH: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
const AUTHORED: u64 = 1_000;
const VALID_UNTIL: u64 = 5_000;
const NOW: u64 = 2_000;

fn authority() -> PartyIdV1 {
    PartyIdV1([1; 32])
}

fn foreign_authority() -> PartyIdV1 {
    PartyIdV1([8; 32])
}

fn reader() -> PartyIdV1 {
    PartyIdV1([2; 32])
}

fn endpoint(label: &str) -> ExposureEndpointIdV1 {
    ExposureEndpointIdV1::derive(authority(), label).unwrap()
}

fn entry_with(store_hash: &str, bytes: &[u8], signature: &str) -> NixCacheEntryV1 {
    let digest = ContentDigestV1::compute(DigestAlgorithmV1::Sha256, bytes);
    NixCacheEntryV1::new(
        &format!("/nix/store/{store_hash}-cf07a-demo"),
        digest,
        bytes.len() as u64,
        vec![],
        None,
        vec![signature],
        None,
    )
    .unwrap()
}

fn entry() -> NixCacheEntryV1 {
    entry_with(STORE_HASH, b"nar-a", "cache.example-1:AAAA==")
}

fn catalog(entries: Vec<NixCacheEntryV1>) -> NixCacheCatalogV1 {
    NixCacheCatalogV1::new(entries).unwrap()
}

fn strict_policy(endpoint: ExposureEndpointIdV1) -> RemoteExposurePolicyV1 {
    RemoteExposurePolicyV1::strict(endpoint, 10_000, 10_000).unwrap()
}

fn grant(
    entry: &NixCacheEntryV1,
    endpoint: ExposureEndpointIdV1,
    audience: ExposureAudienceV1,
    assurance: ExposureAssuranceV1,
) -> RemoteExposureGrantEvidenceV1 {
    RemoteExposureGrantEvidenceV1::for_entry(
        authority(),
        endpoint,
        entry,
        audience,
        AUTHORED,
        AUTHORED,
        VALID_UNTIL,
        assurance,
    )
    .unwrap()
}

fn complete() -> ExposureEvidenceCoverageV1 {
    ExposureEvidenceCoverageV1::CompleteForAuthority
}

#[test]
fn endpoint_and_reader_identities_are_fail_closed() {
    let committed = endpoint("edge-jhb-1");
    assert_eq!(
        ExposureEndpointIdV1::from_stable_id(
            authority(),
            "edge-jhb-1",
            committed.stable_id(),
        )
        .unwrap(),
        committed
    );
    assert!(matches!(
        ExposureEndpointIdV1::from_stable_id(
            authority(),
            "edge-jhb-1",
            StableIdV1::ZERO,
        ),
        Err(ExposureErrorV1::ZeroEndpoint)
    ));
    assert!(matches!(
        ExposureEndpointIdV1::from_stable_id(
            PartyIdV1::ZERO,
            "edge-jhb-1",
            committed.stable_id(),
        ),
        Err(ExposureErrorV1::ZeroAuthority)
    ));
    assert!(matches!(
        ExposureEndpointIdV1::from_stable_id(
            foreign_authority(),
            "edge-jhb-1",
            committed.stable_id(),
        ),
        Err(ExposureErrorV1::EndpointCommitmentMismatch)
    ));
    assert!(matches!(
        ExposureEndpointIdV1::from_stable_id(
            authority(),
            "edge-cpt-1",
            committed.stable_id(),
        ),
        Err(ExposureErrorV1::EndpointCommitmentMismatch)
    ));
    assert!(matches!(
        RemoteReaderV1::authenticated(PartyIdV1::ZERO, []),
        Err(ExposureErrorV1::ZeroReaderPrincipal)
    ));
    assert!(matches!(
        RemoteReaderV1::authenticated(reader(), [StableIdV1::ZERO]),
        Err(ExposureErrorV1::ZeroReaderGroup)
    ));
}

#[test]
fn grant_issuer_must_own_the_endpoint_scope() {
    let entry = entry();
    let foreign_endpoint = ExposureEndpointIdV1::derive(foreign_authority(), "edge-jhb-1").unwrap();
    let error = RemoteExposureGrantEvidenceV1::for_entry(
        authority(),
        foreign_endpoint,
        &entry,
        ExposureAudienceV1::Public,
        AUTHORED,
        AUTHORED,
        VALID_UNTIL,
        ExposureAssuranceV1::CryptographicallyVerified,
    )
    .unwrap_err();
    assert!(matches!(error, ExposureErrorV1::EndpointAuthorityMismatch));
}

#[test]
fn strict_policy_refuses_partial_authority_coverage() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let error = project_remote_exposure_v1(
        &catalog(vec![entry]),
        &[grant],
        &[],
        NOW,
        ExposureEvidenceCoverageV1::Partial,
        &strict_policy(endpoint),
    )
    .unwrap_err();
    assert!(matches!(error, ExposureErrorV1::IncompleteCoverage));
}

#[test]
fn self_claimed_grant_cannot_cross_strict_remote_boundary() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::SelfClaimed,
    );
    let grant_id = grant.id();
    let snapshot = project_remote_exposure_v1(
        &catalog(vec![entry]),
        &[grant],
        &[],
        NOW,
        complete(),
        &strict_policy(endpoint),
    )
    .unwrap();
    assert!(snapshot.is_empty());
    assert!(snapshot.diagnostics().contains(
        &RemoteExposureDiagnosticV1::GrantExcluded {
            grant_id,
            reason: GrantExclusionReasonV1::AssuranceTooLow,
        }
    ));
}

#[test]
fn cryptographically_verified_public_grant_exposes_to_anonymous_reader() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let snapshot = project_remote_exposure_v1(
        &catalog(vec![entry.clone()]),
        &[grant],
        &[],
        NOW,
        complete(),
        &strict_policy(endpoint),
    )
    .unwrap();
    let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
    assert_eq!(snapshot.len(), 1);
    assert_eq!(snapshot.authority(), authority());
    assert_eq!(
        snapshot
            .entry_for_reader(&hash, &RemoteReaderV1::anonymous())
            .unwrap(),
        &entry
    );
}

#[test]
fn principal_grant_does_not_leak_to_anonymous_or_wrong_reader() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::AuthenticatedPrincipal(reader()),
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let snapshot = project_remote_exposure_v1(
        &catalog(vec![entry]),
        &[grant],
        &[],
        NOW,
        complete(),
        &strict_policy(endpoint),
    )
    .unwrap();
    let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
    assert!(
        snapshot
            .entry_for_reader(&hash, &RemoteReaderV1::anonymous())
            .is_none()
    );
    let wrong = RemoteReaderV1::authenticated(PartyIdV1([3; 32]), []).unwrap();
    assert!(snapshot.entry_for_reader(&hash, &wrong).is_none());
    let correct = RemoteReaderV1::authenticated(reader(), []).unwrap();
    assert!(snapshot.entry_for_reader(&hash, &correct).is_some());
}

#[test]
fn group_grant_requires_authenticated_group_fact() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let group = StableIdV1([9; 32]);
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::AuthenticatedGroup(group),
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let snapshot = project_remote_exposure_v1(
        &catalog(vec![entry]),
        &[grant],
        &[],
        NOW,
        complete(),
        &strict_policy(endpoint),
    )
    .unwrap();
    let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
    let without = RemoteReaderV1::authenticated(reader(), []).unwrap();
    let with = RemoteReaderV1::authenticated(reader(), [group]).unwrap();
    assert!(snapshot.entry_for_reader(&hash, &without).is_none());
    assert!(snapshot.entry_for_reader(&hash, &with).is_some());
}

#[test]
fn effective_revocation_removes_exposure() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let grant_id = grant.id();
    let revocation = RemoteExposureRevocationEvidenceV1::new(
        grant_id,
        authority(),
        1_500,
        1_500,
        ExposureAssuranceV1::CryptographicallyVerified,
    )
    .unwrap();
    let snapshot = project_remote_exposure_v1(
        &catalog(vec![entry]),
        &[grant],
        &[revocation],
        NOW,
        complete(),
        &strict_policy(endpoint),
    )
    .unwrap();
    assert!(snapshot.is_empty());
    assert!(snapshot.diagnostics().contains(
        &RemoteExposureDiagnosticV1::GrantExcluded {
            grant_id,
            reason: GrantExclusionReasonV1::Revoked,
        }
    ));
}

#[test]
fn below_policy_revocation_cannot_downgrade_verified_exposure() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let weak = RemoteExposureRevocationEvidenceV1::new(
        grant.id(),
        authority(),
        1_500,
        1_500,
        ExposureAssuranceV1::SelfClaimed,
    )
    .unwrap();
    let cat = catalog(vec![entry]);
    let policy = strict_policy(endpoint);
    let without = project_remote_exposure_v1(
        &cat,
        &[grant.clone()],
        &[],
        NOW,
        complete(),
        &policy,
    )
    .unwrap();
    let with_weak = project_remote_exposure_v1(
        &cat,
        &[grant],
        &[weak],
        NOW,
        complete(),
        &policy,
    )
    .unwrap();
    assert_eq!(without, with_weak);
    assert_eq!(with_weak.len(), 1);
}

#[test]
fn weaker_but_policy_admissible_revocation_is_unresolved_and_fails_closed() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let grant_id = grant.id();
    let revocation = RemoteExposureRevocationEvidenceV1::new(
        grant_id,
        authority(),
        1_500,
        1_500,
        ExposureAssuranceV1::OperatorVerified,
    )
    .unwrap();
    let policy = RemoteExposurePolicyV1::new(
        endpoint,
        ExposureAssuranceV1::OperatorVerified,
        10_000,
        10_000,
        true,
        true,
        true,
        true,
    )
    .unwrap();
    let snapshot = project_remote_exposure_v1(
        &catalog(vec![entry]),
        &[grant],
        &[revocation],
        NOW,
        complete(),
        &policy,
    )
    .unwrap();
    assert!(snapshot.is_empty());
    assert!(snapshot.diagnostics().contains(
        &RemoteExposureDiagnosticV1::GrantExcluded {
            grant_id,
            reason: GrantExclusionReasonV1::RevocationUnresolved,
        }
    ));
}

#[test]
fn future_revocation_is_causally_invisible_to_earlier_replay() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let future = RemoteExposureRevocationEvidenceV1::new(
        grant.id(),
        authority(),
        3_000,
        3_000,
        ExposureAssuranceV1::CryptographicallyVerified,
    )
    .unwrap();
    let cat = catalog(vec![entry]);
    let without = project_remote_exposure_v1(
        &cat,
        &[grant.clone()],
        &[],
        NOW,
        complete(),
        &strict_policy(endpoint),
    )
    .unwrap();
    let with_future = project_remote_exposure_v1(
        &cat,
        &[grant],
        &[future],
        NOW,
        complete(),
        &strict_policy(endpoint),
    )
    .unwrap();
    assert_eq!(without, with_future);
}

#[test]
fn endpoint_grant_cannot_be_replayed_at_another_edge() {
    let entry = entry();
    let edge_a = endpoint("edge-jhb-1");
    let edge_b = endpoint("edge-cpt-1");
    let grant = grant(
        &entry,
        edge_a,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let grant_id = grant.id();
    let snapshot = project_remote_exposure_v1(
        &catalog(vec![entry]),
        &[grant],
        &[],
        NOW,
        complete(),
        &strict_policy(edge_b),
    )
    .unwrap();
    assert!(snapshot.is_empty());
    assert!(snapshot.diagnostics().contains(
        &RemoteExposureDiagnosticV1::GrantExcluded {
            grant_id,
            reason: GrantExclusionReasonV1::EndpointMismatch,
        }
    ));
}

#[test]
fn grant_is_bound_to_exact_nix_publication_metadata() {
    let original = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &original,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let grant_id = grant.id();
    let changed = entry_with(STORE_HASH, b"nar-a", "other-cache-1:BBBB==");
    assert_ne!(
        NixExposureObjectIdV1::from_entry(&original).unwrap(),
        NixExposureObjectIdV1::from_entry(&changed).unwrap()
    );
    let snapshot = project_remote_exposure_v1(
        &catalog(vec![changed]),
        &[grant],
        &[],
        NOW,
        complete(),
        &strict_policy(endpoint),
    )
    .unwrap();
    assert!(snapshot.is_empty());
    assert!(snapshot.diagnostics().contains(
        &RemoteExposureDiagnosticV1::GrantExcluded {
            grant_id,
            reason: GrantExclusionReasonV1::CatalogObjectChanged,
        }
    ));
}

#[test]
fn expiry_is_half_open() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let grant_id = grant.id();
    let snapshot = project_remote_exposure_v1(
        &catalog(vec![entry]),
        &[grant],
        &[],
        VALID_UNTIL,
        complete(),
        &strict_policy(endpoint),
    )
    .unwrap();
    assert!(snapshot.is_empty());
    assert!(snapshot.diagnostics().contains(
        &RemoteExposureDiagnosticV1::GrantExcluded {
            grant_id,
            reason: GrantExclusionReasonV1::Expired,
        }
    ));
}

#[test]
fn stale_grant_fails_closed_even_if_validity_window_remains_open() {
    let entry = entry();
    let endpoint = endpoint("edge-jhb-1");
    let grant = grant(
        &entry,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let grant_id = grant.id();
    let policy = RemoteExposurePolicyV1::strict(endpoint, 10_000, 500).unwrap();
    let snapshot = project_remote_exposure_v1(
        &catalog(vec![entry]),
        &[grant],
        &[],
        NOW,
        complete(),
        &policy,
    )
    .unwrap();
    assert!(snapshot.is_empty());
    assert!(snapshot.diagnostics().contains(
        &RemoteExposureDiagnosticV1::GrantExcluded {
            grant_id,
            reason: GrantExclusionReasonV1::EvidenceTooOld,
        }
    ));
}

#[test]
fn projection_is_input_order_invariant() {
    let first = entry();
    let second = entry_with(OTHER_HASH, b"nar-b", "cache.example-1:BBBB==");
    let endpoint = endpoint("edge-jhb-1");
    let group = StableIdV1([7; 32]);
    let g1 = grant(
        &first,
        endpoint,
        ExposureAudienceV1::Public,
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let g2 = grant(
        &second,
        endpoint,
        ExposureAudienceV1::AuthenticatedGroup(group),
        ExposureAssuranceV1::CryptographicallyVerified,
    );
    let cat = catalog(vec![first, second]);
    let policy = strict_policy(endpoint);
    let a = project_remote_exposure_v1(
        &cat,
        &[g1.clone(), g2.clone()],
        &[],
        NOW,
        complete(),
        &policy,
    )
    .unwrap();
    let b = project_remote_exposure_v1(
        &cat,
        &[g2, g1],
        &[],
        NOW,
        complete(),
        &policy,
    )
    .unwrap();
    assert_eq!(a, b);
    assert_eq!(a.id(), b.id());
}
