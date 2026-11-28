use creator_asset_passport_core::{
    AssetId, AssetKind, AssetRegistry, ClaimState, ExternalIdentifier, GrantId, PartyId,
    PermissionDecision, PermissionDenial, RightsPool, SplitAssessment, SplitClaim,
    SplitCompleteness, UsageGrant, UseAction, UseRequest, assess_split, evaluate_usage,
};

fn asset(value: &str) -> AssetId {
    AssetId::new(value).unwrap()
}

fn party(value: &str) -> PartyId {
    PartyId::new(value).unwrap()
}

fn make_grant(
    asset_id: AssetId,
    action: UseAction,
    purpose: &str,
    valid_from: u64,
    expires_at: Option<u64>,
) -> UsageGrant {
    UsageGrant::new(
        GrantId::new(format!("grant-{purpose}")).unwrap(),
        asset_id,
        party("studio"),
        action,
        purpose,
        party("creator"),
        "evidence://creator-approved",
        valid_from,
        expires_at,
    )
    .unwrap()
}

fn registry_with_recording(id: &AssetId) -> AssetRegistry {
    let mut registry = AssetRegistry::new();
    registry
        .register_asset(
            id.clone(),
            AssetKind::SoundRecording,
            Some("Demo recording".into()),
        )
        .unwrap();
    registry
}

#[test]
fn confirmed_contribution_split_is_not_a_model_training_grant() {
    let recording = asset("recording-1");
    let claims = vec![
        SplitClaim::new(
            recording.clone(),
            party("creator"),
            RightsPool::MasterRecording,
            Some(10_000),
            ClaimState::Confirmed,
            "evidence://split",
        )
        .unwrap(),
    ];
    assert!(matches!(
        assess_split(
            &recording,
            RightsPool::MasterRecording,
            &claims,
            SplitCompleteness::ClaimedComplete
        )
        .unwrap(),
        SplitAssessment::CompleteClaimConsistent { .. }
    ));

    let registry = registry_with_recording(&recording);
    let request = UseRequest::new(
        recording,
        party("studio"),
        UseAction::TrainModel,
        "soundtrack-model-v1",
        100,
    )
    .unwrap();
    assert_eq!(
        evaluate_usage(&registry, &[], &request),
        PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
    );
}

#[test]
fn external_recording_identifier_does_not_authorize_distribution() {
    let recording = asset("recording-1");
    let mut registry = registry_with_recording(&recording);
    registry
        .attach_identifier(
            &recording,
            ExternalIdentifier::Isrc("US-ABC-23-00001".into()),
        )
        .unwrap();
    let request = UseRequest::new(
        recording,
        party("distributor"),
        UseAction::Distribute,
        "single-release",
        100,
    )
    .unwrap();
    assert_eq!(
        evaluate_usage(&registry, &[], &request),
        PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
    );
}

#[test]
fn grant_for_metadata_analysis_does_not_authorize_voice_synthesis() {
    let recording = asset("recording-1");
    let registry = registry_with_recording(&recording);
    let metadata_grant = make_grant(
        recording.clone(),
        UseAction::AnalyzeMetadata,
        "catalog-cleanup",
        0,
        None,
    );
    let request = UseRequest::new(
        recording,
        party("studio"),
        UseAction::SynthesizeVoiceOrLikeness,
        "catalog-cleanup",
        100,
    )
    .unwrap();
    assert_eq!(
        evaluate_usage(&registry, &[metadata_grant], &request),
        PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
    );
}

#[test]
fn grant_is_not_inherited_from_another_recording() {
    let source = asset("source-recording");
    let derivative = asset("derived-recording");
    let mut registry = registry_with_recording(&source);
    registry
        .register_asset(derivative.clone(), AssetKind::SoundRecording, None)
        .unwrap();
    let source_grant = make_grant(source, UseAction::Distribute, "release-a", 0, None);
    let derivative_request = UseRequest::new(
        derivative,
        party("studio"),
        UseAction::Distribute,
        "release-a",
        100,
    )
    .unwrap();
    assert_eq!(
        evaluate_usage(&registry, &[source_grant], &derivative_request),
        PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
    );
}

#[test]
fn purpose_is_part_of_the_permission_scope() {
    let recording = asset("recording-1");
    let registry = registry_with_recording(&recording);
    let grant = make_grant(
        recording.clone(),
        UseAction::Distribute,
        "release-a",
        0,
        None,
    );
    let other_purpose = UseRequest::new(
        recording,
        party("studio"),
        UseAction::Distribute,
        "advertising-campaign-b",
        100,
    )
    .unwrap();
    assert_eq!(
        evaluate_usage(&registry, &[grant], &other_purpose),
        PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
    );
}

#[test]
fn split_claims_cannot_be_combined_across_rights_pools() {
    let work = asset("work-1");
    let claims = vec![
        SplitClaim::new(
            work.clone(),
            party("writer"),
            RightsPool::Composition,
            Some(10_000),
            ClaimState::Confirmed,
            "evidence://work-split",
        )
        .unwrap(),
    ];
    assert_eq!(
        assess_split(
            &work,
            RightsPool::MasterRecording,
            &claims,
            SplitCompleteness::ClaimedComplete
        ),
        Err(creator_asset_passport_core::PassportError::ClaimAssetOrPoolMismatch)
    );
}

#[test]
fn an_unknown_share_is_reported_not_distributed_by_normalization() {
    let work = asset("work-1");
    let claims = vec![
        SplitClaim::new(
            work.clone(),
            party("writer-a"),
            RightsPool::Composition,
            Some(6_000),
            ClaimState::Confirmed,
            "evidence://share-a",
        )
        .unwrap(),
        SplitClaim::new(
            work.clone(),
            party("writer-b"),
            RightsPool::Composition,
            None,
            ClaimState::Confirmed,
            "evidence://share-b",
        )
        .unwrap(),
    ];
    assert_eq!(
        assess_split(
            &work,
            RightsPool::Composition,
            &claims,
            SplitCompleteness::Partial
        )
        .unwrap(),
        SplitAssessment::Partial {
            known_basis_points: 6_000,
            residual_basis_points_not_attributed: 4_000,
            unknown_share_claims: 1,
        }
    );
}

#[test]
fn expiry_is_exclusive_and_never_becomes_an_active_grant_again() {
    let recording = asset("recording-1");
    let registry = registry_with_recording(&recording);
    let grant = make_grant(
        recording.clone(),
        UseAction::Publish,
        "release-a",
        10,
        Some(20),
    );
    let at_expiry = UseRequest::new(
        recording,
        party("studio"),
        UseAction::Publish,
        "release-a",
        20,
    )
    .unwrap();
    assert_eq!(
        evaluate_usage(&registry, &[grant], &at_expiry),
        PermissionDecision::Denied(PermissionDenial::NoActiveGrantForExactScope)
    );
}

#[test]
fn revocation_is_effective_at_its_timestamp_without_rewriting_past_results() {
    let recording = asset("recording-1");
    let registry = registry_with_recording(&recording);
    let grant = make_grant(recording.clone(), UseAction::Publish, "release-a", 10, None);
    let revoked = grant.revoked_at(20).unwrap();
    let before = UseRequest::new(
        recording.clone(),
        party("studio"),
        UseAction::Publish,
        "release-a",
        19,
    )
    .unwrap();
    let at_revoke = UseRequest::new(
        recording,
        party("studio"),
        UseAction::Publish,
        "release-a",
        20,
    )
    .unwrap();
    assert!(matches!(
        evaluate_usage(&registry, std::slice::from_ref(&revoked), &before),
        PermissionDecision::Allowed { .. }
    ));
    assert_eq!(
        evaluate_usage(&registry, std::slice::from_ref(&revoked), &at_revoke),
        PermissionDecision::Denied(PermissionDenial::NoActiveGrantForExactScope)
    );

    // A realistic append-only history can contain both the grant and its
    // later revocation. Neither arrival order may resurrect the original grant.
    assert_eq!(
        evaluate_usage(&registry, &[grant.clone(), revoked.clone()], &at_revoke),
        PermissionDecision::Denied(PermissionDenial::NoActiveGrantForExactScope)
    );
    assert_eq!(
        evaluate_usage(&registry, &[revoked, grant], &at_revoke),
        PermissionDecision::Denied(PermissionDenial::NoActiveGrantForExactScope)
    );
}

#[test]
fn an_unregistered_asset_is_never_authorized_by_a_matching_grant() {
    let unknown = asset("unknown-recording");
    let grant = make_grant(unknown.clone(), UseAction::Publish, "release-a", 0, None);
    let request =
        UseRequest::new(unknown, party("studio"), UseAction::Publish, "release-a", 1).unwrap();
    assert_eq!(
        evaluate_usage(&AssetRegistry::new(), &[grant], &request),
        PermissionDecision::Denied(PermissionDenial::AssetNotRegistered)
    );
}

#[test]
fn external_identifier_collision_is_visible_and_not_auto_merged() {
    let first = asset("recording-1");
    let second = asset("recording-2");
    let mut registry = registry_with_recording(&first);
    registry
        .register_asset(second.clone(), AssetKind::SoundRecording, None)
        .unwrap();
    registry
        .attach_identifier(&first, ExternalIdentifier::Isrc("USABC2300001".into()))
        .unwrap();
    assert_eq!(
        registry.attach_identifier(&second, ExternalIdentifier::Isrc("US-ABC-23-00001".into())),
        Err(creator_asset_passport_core::PassportError::DuplicateExternalIdentifier)
    );
    assert_eq!(registry.len(), 2);
}
