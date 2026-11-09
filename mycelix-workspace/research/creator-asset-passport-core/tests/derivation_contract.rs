use creator_asset_passport_core::{
    AssetId, AssetKind, AssetRegistry, CreativeAssetPassportEnvelope,
    DerivationError, DerivationGraph, DerivationId, DerivationRecord, DerivedPermissionDecision,
    EventId, GrantEvent, GrantHistory, GrantHistoryEnvelope, GrantId, PartyId,
    PermissionDenial, UsageGrant, UseAction, UseRequest, evaluate_derived_usage,
};

fn asset(value: &str) -> AssetId {
    AssetId::new(value).unwrap()
}

fn party(value: &str) -> PartyId {
    PartyId::new(value).unwrap()
}

fn register(registry: &mut AssetRegistry, value: &str, kind: AssetKind) -> AssetId {
    let id = asset(value);
    registry.register_asset(id.clone(), kind, None).unwrap();
    id
}

fn grant(
    asset_id: AssetId,
    grantee: &str,
    action: UseAction,
    purpose: &str,
    grant_id: &str,
) -> UsageGrant {
    UsageGrant::new(
        GrantId::new(grant_id).unwrap(),
        asset_id,
        party(grantee),
        action,
        purpose,
        party("artist"),
        format!("evidence://{grant_id}"),
        1,
        None,
    )
    .unwrap()
}

fn derivation(
    id: &str,
    output: AssetId,
    sources: Vec<AssetId>,
    operation: UseAction,
    purpose: &str,
    actor: &str,
) -> DerivationRecord {
    DerivationRecord::new(
        DerivationId::new(id).unwrap(),
        output,
        sources,
        party(actor),
        operation,
        purpose,
        10,
        "a".repeat(64),
        format!("evidence://{id}"),
    )
    .unwrap()
}

#[test]
fn derivative_creation_requires_an_exact_grant_for_every_source() {
    let mut registry = AssetRegistry::new();
    let source_a = register(&mut registry, "capture-a", AssetKind::DigitalRepresentation);
    let source_b = register(&mut registry, "capture-b", AssetKind::DigitalRepresentation);
    let output = register(&mut registry, "embedding-ab", AssetKind::DerivedArtifact);

    let grants = vec![grant(
        source_a.clone(),
        "embedding-service",
        UseAction::CreateEmbedding,
        "collection-discovery-v1",
        "grant-source-a-embed",
    )];
    let mut graph = DerivationGraph::new();
    let result = graph.record_authorized(
        &registry,
        &grants,
        derivation(
            "derivation-ab",
            output,
            vec![source_a, source_b],
            UseAction::CreateEmbedding,
            "collection-discovery-v1",
            "embedding-service",
        ),
    );

    assert!(matches!(
        result,
        Err(DerivationError::SourceNotAuthorized {
            reason: PermissionDenial::NoExplicitGrant,
            ..
        })
    ));
    assert!(graph.is_empty(), "a denied derivation must not mutate the graph");
}

#[test]
fn derived_retrieval_requires_explicit_grants_on_output_and_all_ancestors() {
    let mut registry = AssetRegistry::new();
    let source_a = register(&mut registry, "capture-a", AssetKind::DigitalRepresentation);
    let source_b = register(&mut registry, "capture-b", AssetKind::DigitalRepresentation);
    let output = register(&mut registry, "embedding-ab", AssetKind::DerivedArtifact);

    let mut grants = vec![
        grant(source_a.clone(), "embedding-service", UseAction::CreateEmbedding, "discovery-v1", "create-a"),
        grant(source_b.clone(), "embedding-service", UseAction::CreateEmbedding, "discovery-v1", "create-b"),
    ];
    let mut graph = DerivationGraph::new();
    graph.record_authorized(
        &registry,
        &grants,
        derivation(
            "embedding-ab-event",
            output.clone(),
            vec![source_a.clone(), source_b.clone()],
            UseAction::CreateEmbedding,
            "discovery-v1",
            "embedding-service",
        ),
    ).unwrap();

    let request = UseRequest::new(
        output.clone(),
        party("curator"),
        UseAction::Retrieve,
        "collection-search",
        50,
    ).unwrap();
    grants.push(grant(
        output.clone(),
        "curator",
        UseAction::Retrieve,
        "collection-search",
        "retrieve-output",
    ));
    grants.push(grant(
        source_a.clone(),
        "curator",
        UseAction::Retrieve,
        "collection-search",
        "retrieve-source-a",
    ));

    assert_eq!(
        evaluate_derived_usage(&registry, &graph, &grants, &request).unwrap(),
        DerivedPermissionDecision::Denied {
            asset_id: source_b.clone(),
            reason: PermissionDenial::NoExplicitGrant,
        }
    );

    grants.push(grant(
        source_b.clone(),
        "curator",
        UseAction::Retrieve,
        "collection-search",
        "retrieve-source-b",
    ));
    match evaluate_derived_usage(&registry, &graph, &grants, &request).unwrap() {
        DerivedPermissionDecision::Allowed { authorization_grants } => {
            assert_eq!(authorization_grants.len(), 3);
            assert!(authorization_grants.iter().any(|item| item.resource_asset_id == output));
            assert!(authorization_grants.iter().any(|item| item.resource_asset_id == source_a));
            assert!(authorization_grants.iter().any(|item| item.resource_asset_id == source_b));
        }
        other => panic!("expected all three scoped grants to authorize, got {other:?}"),
    }
}

#[test]
fn one_derivative_operation_does_not_authorize_a_different_operation() {
    let mut registry = AssetRegistry::new();
    let source = register(&mut registry, "source-image", AssetKind::DigitalRepresentation);
    let output = register(&mut registry, "ocr-text", AssetKind::DerivedArtifact);
    let grants = vec![grant(
        source.clone(),
        "processor",
        UseAction::CreateEmbedding,
        "research-index",
        "embedding-only",
    )];

    let mut graph = DerivationGraph::new();
    let result = graph.record_authorized(
        &registry,
        &grants,
        derivation(
            "ocr-event",
            output,
            vec![source],
            UseAction::RunOcr,
            "research-index",
            "processor",
        ),
    );
    assert!(matches!(
        result,
        Err(DerivationError::SourceNotAuthorized {
            reason: PermissionDenial::NoExplicitGrant,
            ..
        })
    ));
    assert!(graph.is_empty());
}

#[test]
fn derivation_graph_rejects_cycles_without_mutating_prior_lineage() {
    let mut registry = AssetRegistry::new();
    let original = register(&mut registry, "original", AssetKind::PhysicalArtwork);
    let derived = register(&mut registry, "thumbnail", AssetKind::DerivedArtifact);
    let grants = vec![
        grant(original.clone(), "thumbnailer", UseAction::GenerateThumbnail, "catalogue-preview", "thumb-source"),
        grant(derived.clone(), "thumbnailer", UseAction::GenerateThumbnail, "catalogue-preview", "thumb-derived"),
    ];

    let mut graph = DerivationGraph::new();
    graph.record_authorized(
        &registry,
        &grants,
        derivation(
            "thumbnail-event",
            derived.clone(),
            vec![original.clone()],
            UseAction::GenerateThumbnail,
            "catalogue-preview",
            "thumbnailer",
        ),
    ).unwrap();

    let cycle = derivation(
        "cycle-event",
        original.clone(),
        vec![derived],
        UseAction::GenerateThumbnail,
        "catalogue-preview",
        "thumbnailer",
    );
    assert_eq!(
        graph.record_authorized(&registry, &grants, cycle),
        Err(DerivationError::CycleDetected)
    );
    assert_eq!(graph.len(), 1);
    assert_eq!(graph.ancestors_of(&original).unwrap(), Vec::<AssetId>::new());
}

#[test]
fn shared_ancestors_are_deduplicated_in_deterministic_lineage() {
    let mut registry = AssetRegistry::new();
    let root = register(&mut registry, "root-capture", AssetKind::DigitalRepresentation);
    let left = register(&mut registry, "left-crop", AssetKind::DerivedArtifact);
    let right = register(&mut registry, "right-crop", AssetKind::DerivedArtifact);
    let composite = register(&mut registry, "composite", AssetKind::DerivedArtifact);
    let grants = vec![
        grant(root.clone(), "processor", UseAction::GenerateThumbnail, "curatorial-view", "root-grant"),
        grant(left.clone(), "processor", UseAction::GenerateThumbnail, "curatorial-view", "left-grant"),
        grant(right.clone(), "processor", UseAction::GenerateThumbnail, "curatorial-view", "right-grant"),
    ];

    let mut graph = DerivationGraph::new();
    graph.record_authorized(
        &registry,
        &grants,
        derivation("left-event", left.clone(), vec![root.clone()], UseAction::GenerateThumbnail, "curatorial-view", "processor"),
    ).unwrap();
    graph.record_authorized(
        &registry,
        &grants,
        derivation("right-event", right.clone(), vec![root.clone()], UseAction::GenerateThumbnail, "curatorial-view", "processor"),
    ).unwrap();
    graph.record_authorized(
        &registry,
        &grants,
        derivation("composite-event", composite.clone(), vec![left.clone(), right.clone()], UseAction::GenerateThumbnail, "curatorial-view", "processor"),
    ).unwrap();

    let expected_lineage = vec![left.clone(), right.clone(), root.clone()];
    assert_eq!(graph.ancestors_of(&composite).unwrap(), expected_lineage);
    let record = graph.get(&composite).unwrap();
    let authorization_ids: Vec<_> = record.authorization_grants()
        .iter()
        .map(|item| item.resource_asset_id.clone())
        .collect();
    assert_eq!(authorization_ids, expected_lineage);
}

#[test]
fn non_derivation_operations_cannot_be_recorded_as_transformations() {
    let result = DerivationRecord::new(
        DerivationId::new("invalid-derivation").unwrap(),
        asset("output"),
        vec![asset("source")],
        party("processor"),
        UseAction::Publish,
        "release",
        10,
        "b".repeat(64),
        "evidence://invalid",
    );
    assert_eq!(result, Err(DerivationError::NonDerivationOperation));
}

#[test]
fn physical_artwork_asset_kinds_round_trip_in_the_portable_passport() {
    let kinds = [
        AssetKind::PhysicalArtwork,
        AssetKind::ArtworkEdition,
        AssetKind::ArtworkComponent,
        AssetKind::DigitalRepresentation,
        AssetKind::DocumentationAsset,
        AssetKind::DerivedArtifact,
    ];
    let mut registry = AssetRegistry::new();
    for (index, kind) in kinds.iter().copied().enumerate() {
        registry.register_asset(asset(&format!("art-resource-{index}")), kind, None).unwrap();
    }

    let envelope = CreativeAssetPassportEnvelope::from_parts(
        &registry,
        &[],
        &GrantHistory::new(),
    ).unwrap();
    let bytes = envelope.canonical_bytes().unwrap();
    let decoded = CreativeAssetPassportEnvelope::from_bytes(&bytes).unwrap();

    for (index, expected_kind) in kinds.iter().copied().enumerate() {
        let id = asset(&format!("art-resource-{index}"));
        assert_eq!(decoded.registry().get(&id).unwrap().kind, expected_kind);
    }
}

#[test]
fn new_artwork_use_operations_round_trip_in_portable_grant_history() {
    let actions = [
        UseAction::Capture,
        UseAction::Preserve,
        UseAction::Transform,
        UseAction::Index,
        UseAction::Embed,
        UseAction::Retrieve,
        UseAction::Display,
        UseAction::Export,
        UseAction::EvaluateModel,
        UseAction::GenerateThumbnail,
        UseAction::RunOcr,
        UseAction::CreateCaption,
        UseAction::CreateEmbedding,
        UseAction::ReconstructMesh,
        UseAction::ReconstructPointCloud,
    ];
    let mut history = GrantHistory::new();
    for (index, action) in actions.iter().copied().enumerate() {
        let grant_id = format!("action-grant-{index}");
        let grant = UsageGrant::new(
            GrantId::new(&grant_id).unwrap(),
            asset("artwork-1"),
            party("service"),
            action,
            format!("purpose-{index}"),
            party("artist"),
            format!("evidence://{grant_id}"),
            1,
            None,
        ).unwrap();
        history.append(GrantEvent::issue(
            EventId::new(format!("action-event-{index}")).unwrap(),
            0,
            party("artist"),
            format!("evidence://issue-{index}"),
            grant,
        ).unwrap()).unwrap();
    }

    let envelope = GrantHistoryEnvelope::from_history(&history).unwrap();
    let bytes = envelope.canonical_bytes().unwrap();
    let decoded = GrantHistoryEnvelope::from_bytes(&bytes).unwrap();
    assert_eq!(decoded, envelope);
    assert_eq!(decoded.replay().unwrap(), history);
}
