use creator_asset_passport_core::{
    AssetId, AssetKind, AssetRegistry, ExternalIdentifier, GrantId, PartyId, PermissionDecision,
    PermissionDenial, UsageGrant, UseAction, UseRequest, evaluate_usage,
};

fn asset(value: &str) -> AssetId {
    AssetId::new(value).unwrap()
}

fn party(value: &str) -> PartyId {
    PartyId::new(value).unwrap()
}

#[test]
fn iswc_rejects_malformed_separator_layout() {
    let work = asset("work-1");
    let mut registry = AssetRegistry::new();
    registry
        .register_asset(work.clone(), AssetKind::MusicalWork, None)
        .unwrap();
    assert_eq!(
        registry.attach_identifier(&work, ExternalIdentifier::Iswc("T--034.524.680-1".into())),
        Err(creator_asset_passport_core::PassportError::InvalidIswcSyntax)
    );
}

#[test]
fn isrc_rejects_malformed_separator_layout() {
    let recording = asset("recording-1");
    let mut registry = AssetRegistry::new();
    registry
        .register_asset(recording.clone(), AssetKind::SoundRecording, None)
        .unwrap();
    assert_eq!(
        registry.attach_identifier(
            &recording,
            ExternalIdentifier::Isrc("US--ABC-23-00001".into())
        ),
        Err(creator_asset_passport_core::PassportError::InvalidIsrcSyntax)
    );
}

#[test]
fn explicit_training_consent_is_narrow_to_asset_party_action_and_purpose() {
    let recording = asset("recording-1");
    let mut registry = AssetRegistry::new();
    registry
        .register_asset(recording.clone(), AssetKind::SoundRecording, None)
        .unwrap();
    let grant = UsageGrant::new(
        GrantId::new("training-consent-1").unwrap(),
        recording.clone(),
        party("studio"),
        UseAction::TrainModel,
        "private-demo-model-v1",
        party("creator"),
        "evidence://explicit-training-consent",
        100,
        Some(200),
    )
    .unwrap();

    let exact = UseRequest::new(
        recording.clone(),
        party("studio"),
        UseAction::TrainModel,
        "private-demo-model-v1",
        150,
    )
    .unwrap();
    assert_eq!(
        evaluate_usage(&registry, std::slice::from_ref(&grant), &exact),
        PermissionDecision::Allowed {
            grant_id: GrantId::new("training-consent-1").unwrap()
        }
    );

    let renamed_model = UseRequest::new(
        recording.clone(),
        party("studio"),
        UseAction::TrainModel,
        "general-purpose-model",
        150,
    )
    .unwrap();
    assert_eq!(
        evaluate_usage(&registry, std::slice::from_ref(&grant), &renamed_model),
        PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
    );

    let different_action = UseRequest::new(
        recording,
        party("studio"),
        UseAction::Distribute,
        "private-demo-model-v1",
        150,
    )
    .unwrap();
    assert_eq!(
        evaluate_usage(&registry, &[grant], &different_action),
        PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
    );
}
