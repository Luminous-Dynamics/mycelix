use creator_asset_passport_core::history::{GrantEvent, GrantHistory};
use creator_asset_passport_core::passport_envelope::{
    CreativeAssetPassportEnvelope, PassportEnvelopeError,
};
use creator_asset_passport_core::{
    AssetId, AssetKind, AssetRegistry, ClaimState, EventId, ExternalIdentifier, GrantId, PartyId,
    RightsPool, SplitClaim, UsageGrant, UseAction,
};

fn party(value: &str) -> PartyId {
    PartyId::new(value).unwrap()
}

fn seeded_passport() -> CreativeAssetPassportEnvelope {
    let mut registry = AssetRegistry::new();
    let work = AssetId::new("work-a").unwrap();
    registry
        .register_asset(work.clone(), AssetKind::MusicalWork, Some("Song A".into()))
        .unwrap();
    registry
        .attach_identifier(&work, ExternalIdentifier::Iswc("T-034.524.680-1".into()))
        .unwrap();

    let recording = AssetId::new("recording-a").unwrap();
    registry
        .register_asset(
            recording.clone(),
            AssetKind::SoundRecording,
            Some("Master A".into()),
        )
        .unwrap();
    registry
        .attach_identifier(
            &recording,
            ExternalIdentifier::Isrc("US-ABC-23-00001".into()),
        )
        .unwrap();

    let claims = vec![
        SplitClaim::new(
            work.clone(),
            party("writer-a"),
            RightsPool::Composition,
            Some(6_000),
            ClaimState::Confirmed,
            "split://writer-a",
        )
        .unwrap(),
        SplitClaim::new(
            work,
            party("writer-b"),
            RightsPool::Composition,
            Some(4_000),
            ClaimState::Confirmed,
            "split://writer-b",
        )
        .unwrap(),
    ];

    let grant = UsageGrant::new(
        GrantId::new("grant-a").unwrap(),
        recording,
        party("distributor"),
        UseAction::Distribute,
        "release-a",
        party("creator"),
        "consent://release-a",
        10,
        Some(100),
    )
    .unwrap();
    let issue = GrantEvent::issue(
        EventId::new("event-issue").unwrap(),
        5,
        party("creator"),
        "evidence://issue",
        grant,
    )
    .unwrap();
    let revoke = GrantEvent::revoke(
        EventId::new("event-revoke").unwrap(),
        50,
        party("creator"),
        "evidence://revoke",
        GrantId::new("grant-a").unwrap(),
    )
    .unwrap();
    let history = GrantHistory::replay(&[issue, revoke]).unwrap();

    CreativeAssetPassportEnvelope::from_parts(&registry, &claims, &history).unwrap()
}

#[test]
fn full_snapshot_round_trip_preserves_all_supported_record_families() {
    let passport = seeded_passport();
    let expected_digest = passport.digest_hex().unwrap();
    let bytes = passport.canonical_bytes().unwrap();

    let imported = CreativeAssetPassportEnvelope::from_bytes(&bytes).unwrap();
    assert_eq!(imported, passport);
    assert_eq!(imported.canonical_bytes().unwrap(), bytes);
    assert_eq!(imported.digest_hex().unwrap(), expected_digest);
    assert_eq!(imported.registry().len(), 2);
    assert_eq!(imported.claims().len(), 2);
    assert_eq!(imported.grant_history().unwrap().len(), 2);

    let master = imported
        .registry()
        .get(&AssetId::new("recording-a").unwrap())
        .unwrap();
    assert_eq!(master.identifiers().len(), 1);
    assert_eq!(
        imported
            .grant_history()
            .unwrap()
            .find_grant(&GrantId::new("grant-a").unwrap())
            .unwrap()
            .revocation_timestamp(),
        Some(50)
    );
}

#[test]
fn claim_input_order_does_not_change_canonical_snapshot_bytes() {
    let passport = seeded_passport();
    let assets = passport.registry();
    let history = passport.grant_history().unwrap();
    let mut claims = passport.claims().to_vec();
    claims.reverse();

    let differently_ordered =
        CreativeAssetPassportEnvelope::from_parts(assets, &claims, &history).unwrap();
    assert_eq!(
        differently_ordered.canonical_bytes().unwrap(),
        passport.canonical_bytes().unwrap()
    );
    assert_eq!(
        differently_ordered.digest_hex().unwrap(),
        passport.digest_hex().unwrap()
    );
}

#[test]
fn an_unknown_asset_reference_fails_before_export() {
    let passport = seeded_passport();
    let history = passport.grant_history().unwrap();
    let mut claims = passport.claims().to_vec();
    claims.push(
        SplitClaim::new(
            AssetId::new("not-in-registry").unwrap(),
            party("writer-c"),
            RightsPool::Composition,
            Some(1_000),
            ClaimState::Proposed,
            "split://writer-c",
        )
        .unwrap(),
    );

    assert_eq!(
        CreativeAssetPassportEnvelope::from_parts(passport.registry(), &claims, &history),
        Err(PassportEnvelopeError::InvalidClaimAsset)
    );
}

#[test]
fn changing_asset_metadata_changes_the_snapshot_digest() {
    let passport = seeded_passport();
    let original = passport.digest_hex().unwrap();

    let changed_registry = {
        let mut replacement = AssetRegistry::new();
        let asset = AssetId::new("work-a").unwrap();
        replacement
            .register_asset(
                asset.clone(),
                AssetKind::MusicalWork,
                Some("Different song".into()),
            )
            .unwrap();
        replacement
            .attach_identifier(&asset, ExternalIdentifier::Iswc("T-034.524.680-1".into()))
            .unwrap();
        let recording = AssetId::new("recording-a").unwrap();
        replacement
            .register_asset(
                recording.clone(),
                AssetKind::SoundRecording,
                Some("Master A".into()),
            )
            .unwrap();
        replacement
            .attach_identifier(
                &recording,
                ExternalIdentifier::Isrc("US-ABC-23-00001".into()),
            )
            .unwrap();
        replacement
    };
    let changed = CreativeAssetPassportEnvelope::from_parts(
        &changed_registry,
        passport.claims(),
        &passport.grant_history().unwrap(),
    )
    .unwrap();
    assert_ne!(changed.digest_hex().unwrap(), original);
}

#[test]
fn changing_split_claim_evidence_changes_the_snapshot_digest() {
    let passport = seeded_passport();
    let original = passport.digest_hex().unwrap();
    let history = passport.grant_history().unwrap();
    let mut claims = passport.claims().to_vec();
    claims[0].evidence_ref.push_str("-amended");

    let changed =
        CreativeAssetPassportEnvelope::from_parts(passport.registry(), &claims, &history).unwrap();
    assert_ne!(changed.digest_hex().unwrap(), original);
}

#[test]
fn changing_grant_event_evidence_changes_the_snapshot_digest() {
    let passport = seeded_passport();
    let original = passport.digest_hex().unwrap();

    let grant = UsageGrant::new(
        GrantId::new("grant-a").unwrap(),
        AssetId::new("recording-a").unwrap(),
        party("distributor"),
        UseAction::Distribute,
        "release-a",
        party("creator"),
        "consent://release-a",
        10,
        Some(100),
    )
    .unwrap();
    let issue = GrantEvent::issue(
        EventId::new("event-issue").unwrap(),
        5,
        party("creator"),
        "evidence://amended-issue",
        grant,
    )
    .unwrap();
    let revoke = GrantEvent::revoke(
        EventId::new("event-revoke").unwrap(),
        50,
        party("creator"),
        "evidence://revoke",
        GrantId::new("grant-a").unwrap(),
    )
    .unwrap();
    let changed_history = GrantHistory::replay(&[issue, revoke]).unwrap();
    let changed = CreativeAssetPassportEnvelope::from_parts(
        passport.registry(),
        passport.claims(),
        &changed_history,
    )
    .unwrap();

    assert_ne!(changed.digest_hex().unwrap(), original);
}

#[test]
fn grant_history_targets_must_exist_in_the_exported_asset_registry() {
    let registry = AssetRegistry::new();
    let grant = UsageGrant::new(
        GrantId::new("grant-unbound").unwrap(),
        AssetId::new("absent-recording").unwrap(),
        party("distributor"),
        UseAction::Distribute,
        "release-a",
        party("creator"),
        "consent://release-a",
        10,
        None,
    )
    .unwrap();
    let issue = GrantEvent::issue(
        EventId::new("event-unbound").unwrap(),
        5,
        party("creator"),
        "evidence://issue",
        grant,
    )
    .unwrap();
    let history = GrantHistory::replay(&[issue]).unwrap();

    assert_eq!(
        CreativeAssetPassportEnvelope::from_parts(&registry, &[], &history),
        Err(PassportEnvelopeError::InvalidGrantAsset)
    );
}

#[test]
fn external_identifier_insertion_order_does_not_change_snapshot_bytes() {
    fn registry(reverse: bool) -> AssetRegistry {
        let mut assets = AssetRegistry::new();
        let recording = AssetId::new("recording-order").unwrap();
        assets
            .register_asset(
                recording.clone(),
                AssetKind::SoundRecording,
                Some("Order test".into()),
            )
            .unwrap();
        let a = ExternalIdentifier::Other {
            scheme: "a".into(),
            value: "z".into(),
        };
        let b = ExternalIdentifier::Other {
            scheme: "a:".into(),
            value: "x".into(),
        };
        if reverse {
            assets.attach_identifier(&recording, b).unwrap();
            assets.attach_identifier(&recording, a).unwrap();
        } else {
            assets.attach_identifier(&recording, a).unwrap();
            assets.attach_identifier(&recording, b).unwrap();
        }
        assets
    }

    let history = GrantHistory::new();
    let first = CreativeAssetPassportEnvelope::from_parts(&registry(false), &[], &history).unwrap();
    let second = CreativeAssetPassportEnvelope::from_parts(&registry(true), &[], &history).unwrap();
    assert_eq!(
        first.canonical_bytes().unwrap(),
        second.canonical_bytes().unwrap()
    );
}

#[test]
fn unsupported_snapshot_versions_and_trailing_bytes_are_rejected() {
    let passport = seeded_passport();
    let mut bytes = passport.canonical_bytes().unwrap();
    bytes[b"MYCELIX-CSCP-ASSET-PASSPORT\0".len() + 1] = 2;
    assert_eq!(
        CreativeAssetPassportEnvelope::from_bytes(&bytes),
        Err(PassportEnvelopeError::UnsupportedVersion(2))
    );

    let mut bytes = passport.canonical_bytes().unwrap();
    bytes.push(0);
    assert_eq!(
        CreativeAssetPassportEnvelope::from_bytes(&bytes),
        Err(PassportEnvelopeError::TrailingBytes)
    );
}
