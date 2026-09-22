use pulse_external_bridge_types::*;

fn sha(ch: char) -> String {
    format!("sha256:{}", ch.to_string().repeat(64))
}

fn matrix_base() -> RawExternalBridgeObservationV1 {
    RawExternalBridgeObservationV1 {
        provider_family: ProviderFamily::Matrix,
        provider_profile: MATRIX_PROFILE_V0_1.to_string(),
        provider_event_class: ProviderEventClassV1::MatrixPersistentRoomEvent,
        provider_event_kind: "m.room.message".to_string(),
        provider_event_id: Some("$event:example.org".to_string()),
        provider_delivery: Some(RawProviderDeliveryV1 {
            delivery_id: "txn-1".to_string(),
            delivery_commitment: Some(sha('a')),
        }),
        provider_scope: ProviderScopeV1::Matrix {
            room_id: Some("!room:example.org".to_string()),
            state_key: None,
        },
        provider_actor: Some(RawProviderActorV1 {
            actor_id: "@alice:example.org".to_string(),
            actor_kind: ProviderActorKindV1::MatrixUser,
        }),
        provider_chronology: None,
        content: ContentObservationV1::Present("hello".to_string()),
        relations: vec![],
        attachments: vec![],
        moderation_observations: vec![],
        visibility_observation: Some("private-room".to_string()),
        bridge_origin: None,
        provider_extension_markers: vec![],
        losses: vec![LossMarkerV1::LosslessWithinProjectionProfile],
    }
}

fn discord_base() -> RawExternalBridgeObservationV1 {
    RawExternalBridgeObservationV1 {
        provider_family: ProviderFamily::Discord,
        provider_profile: DISCORD_PROFILE_V0_1.to_string(),
        provider_event_class: ProviderEventClassV1::DiscordMessage,
        provider_event_kind: "MESSAGE_CREATE".to_string(),
        provider_event_id: Some("1001".to_string()),
        provider_delivery: None,
        provider_scope: ProviderScopeV1::Discord {
            guild_id: Some("2001".to_string()),
            channel_id: Some("3001".to_string()),
            thread_id: None,
        },
        provider_actor: Some(RawProviderActorV1 {
            actor_id: "4001".to_string(),
            actor_kind: ProviderActorKindV1::DiscordUser,
        }),
        provider_chronology: None,
        content: ContentObservationV1::Present("hello".to_string()),
        relations: vec![],
        attachments: vec![],
        moderation_observations: vec![],
        visibility_observation: Some("guild-channel".to_string()),
        bridge_origin: None,
        provider_extension_markers: vec![],
        losses: vec![LossMarkerV1::LosslessWithinProjectionProfile],
    }
}

#[test]
fn matrix_state_requires_state_key_presence() {
    let mut raw = matrix_base();
    raw.provider_event_class = ProviderEventClassV1::MatrixStateEvent;
    assert_eq!(
        admit_external_bridge_observation(raw).unwrap_err(),
        BridgeAdmissionError::ProviderEventClassMismatch
    );

    let mut raw = matrix_base();
    raw.provider_event_class = ProviderEventClassV1::MatrixStateEvent;
    raw.provider_scope = ProviderScopeV1::Matrix {
        room_id: Some("!room:example.org".to_string()),
        state_key: Some(String::new()),
    };
    assert!(admit_external_bridge_observation(raw).is_ok());
}

#[test]
fn matrix_txn_is_delivery_not_event_identity() {
    let mut first = matrix_base();
    first.provider_event_id = Some("$same:example.org".to_string());
    first.provider_delivery = Some(RawProviderDeliveryV1 {
        delivery_id: "txn-1".to_string(),
        delivery_commitment: Some(sha('a')),
    });

    let mut second = first.clone();
    second.provider_delivery = Some(RawProviderDeliveryV1 {
        delivery_id: "txn-2".to_string(),
        delivery_commitment: Some(sha('b')),
    });

    let admitted = admit_external_bridge_batch(vec![second, first]).unwrap();
    assert_eq!(admitted.len(), 2);
    assert_eq!(admitted[0].provider_event_id(), Some("$same:example.org"));
    assert_eq!(admitted[1].provider_event_id(), Some("$same:example.org"));
    assert_ne!(
        admitted[0].provider_delivery().unwrap().delivery_id,
        admitted[1].provider_delivery().unwrap().delivery_id
    );
}

#[test]
fn conflicting_matrix_delivery_coordinate_fails_closed() {
    let first = matrix_base();
    let mut second = first.clone();
    second.provider_event_id = Some("$other:example.org".to_string());
    second.provider_delivery.as_mut().unwrap().delivery_commitment = Some(sha('b'));

    assert_eq!(
        admit_external_bridge_batch(vec![first, second]).unwrap_err(),
        BridgeAdmissionError::ConflictingDeliveryCoordinate
    );
}

#[test]
fn discord_scope_preserves_guild_channel_and_thread_independently() {
    let mut raw = discord_base();
    raw.provider_scope = ProviderScopeV1::Discord {
        guild_id: Some("guild".to_string()),
        channel_id: Some("channel".to_string()),
        thread_id: Some("thread".to_string()),
    };
    let admitted = admit_external_bridge_observation(raw).unwrap();
    assert_eq!(
        admitted.provider_scope(),
        &ProviderScopeV1::Discord {
            guild_id: Some("guild".to_string()),
            channel_id: Some("channel".to_string()),
            thread_id: Some("thread".to_string()),
        }
    );
}

#[test]
fn webhook_authorship_is_preserved() {
    let mut raw = discord_base();
    raw.provider_actor = Some(RawProviderActorV1 {
        actor_id: "webhook-1".to_string(),
        actor_kind: ProviderActorKindV1::DiscordWebhook,
    });
    let admitted = admit_external_bridge_observation(raw).unwrap();
    assert_eq!(
        admitted.provider_actor().unwrap().actor_kind,
        ProviderActorKindV1::DiscordWebhook
    );
}

#[test]
fn provider_family_actor_mismatch_rejects() {
    let mut raw = discord_base();
    raw.provider_actor.as_mut().unwrap().actor_kind = ProviderActorKindV1::MatrixUser;
    assert_eq!(
        admit_external_bridge_observation(raw).unwrap_err(),
        BridgeAdmissionError::ProviderScopeMismatch
    );
}

#[test]
fn withheld_content_is_not_empty_content_truth() {
    let mut raw = discord_base();
    raw.content = ContentObservationV1::UnavailableUnderProviderAccessProfile;
    raw.losses = vec![LossMarkerV1::ContentUnavailableUnderProviderAccessProfile];

    let admitted = admit_external_bridge_observation(raw).unwrap();
    assert_eq!(
        admitted.content(),
        &ContentObservationV1::UnavailableUnderProviderAccessProfile
    );
}

#[test]
fn withheld_content_requires_explicit_loss_marker() {
    let mut raw = discord_base();
    raw.content = ContentObservationV1::UnavailableUnderProviderAccessProfile;
    raw.losses = vec![LossMarkerV1::LosslessWithinProjectionProfile];

    assert_eq!(
        admit_external_bridge_observation(raw).unwrap_err(),
        BridgeAdmissionError::MissingRequiredLoss("content_unavailable")
    );
}

#[test]
fn attachment_reference_survives_not_fetched_loss() {
    let mut raw = discord_base();
    raw.attachments = vec![ProviderAttachmentRefV1 {
        provider_attachment_id: "attachment-1".to_string(),
        provider_locator: Some("https://cdn.example.invalid/a".to_string()),
        availability: AttachmentAvailabilityV1::ReferenceOnly,
        content_commitment: None,
    }];
    raw.losses = vec![LossMarkerV1::AttachmentNotFetched];

    let admitted = admit_external_bridge_observation(raw).unwrap();
    assert_eq!(admitted.attachments().len(), 1);
    assert_eq!(
        admitted.attachments()[0].provider_attachment_id,
        "attachment-1"
    );
}

#[test]
fn reference_only_attachment_without_loss_rejects() {
    let mut raw = discord_base();
    raw.attachments = vec![ProviderAttachmentRefV1 {
        provider_attachment_id: "attachment-1".to_string(),
        provider_locator: None,
        availability: AttachmentAvailabilityV1::ReferenceOnly,
        content_commitment: None,
    }];

    assert_eq!(
        admit_external_bridge_observation(raw).unwrap_err(),
        BridgeAdmissionError::MissingRequiredLoss("attachment_not_fetched")
    );
}

#[test]
fn moderation_observation_survives_without_minting_authority() {
    let mut raw = discord_base();
    raw.moderation_observations = vec![ProviderModerationObservationV1 {
        provider_kind: "role".to_string(),
        provider_ref: "role-3001".to_string(),
    }];
    raw.losses = vec![LossMarkerV1::ProviderModerationObservationOnly];

    let admitted = admit_external_bridge_observation(raw).unwrap();
    assert_eq!(admitted.moderation_observations().len(), 1);
    assert_eq!(
        admitted.moderation_observations()[0].provider_ref,
        "role-3001"
    );
}

#[test]
fn unknown_extension_is_retained_as_explicit_loss() {
    let mut raw = matrix_base();
    raw.provider_extension_markers = vec!["org.example.future.unknown".to_string()];
    raw.losses = vec![LossMarkerV1::UnknownProviderExtension];
    assert!(admit_external_bridge_observation(raw).is_ok());
}

#[test]
fn lossless_cannot_coexist_with_loss() {
    let mut raw = discord_base();
    raw.losses = vec![
        LossMarkerV1::LosslessWithinProjectionProfile,
        LossMarkerV1::AudienceSemanticsNotRepresentable,
    ];
    assert_eq!(
        admit_external_bridge_observation(raw).unwrap_err(),
        BridgeAdmissionError::ConflictingLossClaim
    );
}

#[test]
fn every_admitted_observation_has_explicit_loss_classification() {
    let mut raw = discord_base();
    raw.losses.clear();
    assert_eq!(
        admit_external_bridge_observation(raw).unwrap_err(),
        BridgeAdmissionError::MissingRequiredLoss("projection_loss_classification")
    );
}

#[test]
fn bridge_origin_preserves_provider_and_semantic_commitment() {
    let mut raw = discord_base();
    raw.bridge_origin = Some(RawBridgeOriginV1 {
        source_provider: ProviderFamily::Matrix,
        source_provider_event_id: Some("$loop:example.org".to_string()),
        source_semantic_commitment: sha('c'),
    });
    let admitted = admit_external_bridge_observation(raw).unwrap();
    let origin = admitted.bridge_origin().unwrap();
    assert_eq!(origin.source_provider, ProviderFamily::Matrix);
    assert_eq!(
        origin.source_provider_event_id.as_deref(),
        Some("$loop:example.org")
    );
}

#[test]
fn text_equality_does_not_deduplicate_distinct_provider_events() {
    let matrix = matrix_base();
    let discord = discord_base();
    let admitted = admit_external_bridge_batch(vec![matrix, discord]).unwrap();
    assert_eq!(admitted.len(), 2);
    assert_ne!(admitted[0].provider_family(), admitted[1].provider_family());
}

#[test]
fn profile_substitution_fails_closed() {
    let mut raw = discord_base();
    raw.provider_profile = "unreviewed-profile".to_string();
    assert_eq!(
        admit_external_bridge_observation(raw).unwrap_err(),
        BridgeAdmissionError::ProviderProfileMismatch
    );
}

#[test]
fn canonical_batch_order_is_input_permutation_independent() {
    let matrix = matrix_base();
    let discord = discord_base();

    let a = admit_external_bridge_batch(vec![matrix.clone(), discord.clone()]).unwrap();
    let b = admit_external_bridge_batch(vec![discord, matrix]).unwrap();

    assert_eq!(a, b);
    assert_eq!(
        serde_json::to_vec(&a).unwrap(),
        serde_json::to_vec(&b).unwrap()
    );
}

#[test]
fn invalid_bridge_commitment_rejects() {
    let mut raw = discord_base();
    raw.bridge_origin = Some(RawBridgeOriginV1 {
        source_provider: ProviderFamily::Matrix,
        source_provider_event_id: Some("$loop:example.org".to_string()),
        source_semantic_commitment: "sha256:ABC".to_string(),
    });

    assert_eq!(
        admit_external_bridge_observation(raw).unwrap_err(),
        BridgeAdmissionError::InvalidCommitment(
            "bridge_origin_source_semantic_commitment"
        )
    );
}

#[test]
fn raw_json_can_deserialize_but_positive_requires_admission() {
    let raw = discord_base();
    let json = serde_json::to_string(&raw).unwrap();
    let decoded: RawExternalBridgeObservationV1 = serde_json::from_str(&json).unwrap();
    let admitted = admit_external_bridge_observation(decoded).unwrap();
    assert_eq!(admitted.provider_event_id(), Some("1001"));
}

#[test]
fn exact_frozen_corpus_is_bound_into_the_crate() {
    let fixture: serde_json::Value =
        serde_json::from_str(FROZEN_BRIDGE_CORPUS_V0_1).unwrap();
    assert_eq!(
        fixture["profile"].as_str().unwrap(),
        "mycelix:pulse:matrix-discord-bridge-corpus:v0.1"
    );
    assert_eq!(fixture["cases"].as_array().unwrap().len(), 20);
    assert!(FROZEN_PROFILE_V0_1.contains("Native Pulse firewall"));
}
