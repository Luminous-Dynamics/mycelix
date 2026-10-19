use ed25519_dalek::{Signer, SigningKey};
use mycelix_integration_core::{
    CanonicalEncodeV1, ConnectorInstanceId, ContentCommitment, ExecutionAttemptId,
    ExternalObjectRef, ExternalObjectType, ExternalOpaqueId, ExternalOperationKind,
    ExternalSystemId, IdempotencyKey, IntegrationCommand, IntegrationCommandId,
    SemanticProfileId, SideEffectClass,
};
use mycelix_integration_execution_binding::{
    qualify_execution_binding, qualify_provider_profile, ProviderExecutionProfile,
    ProviderIdempotencySemantics, ProviderProfileTrustRoot, ProviderReconciliationMode,
    SignedProviderExecutionProfile, PROFILE_PROTOCOL_V1,
};
use mycelix_integration_runtime::ExecutionClaim;

#[derive(Debug, Clone, PartialEq, Eq)]
struct DemoPayload(u64);

impl CanonicalEncodeV1 for DemoPayload {
    fn canonical_preimage_v1(&self) -> Vec<u8> {
        let mut out = b"DEMO-PAYLOAD\0V1\0".to_vec();
        out.extend_from_slice(&self.0.to_be_bytes());
        out
    }
}

fn command() -> IntegrationCommand<DemoPayload> {
    IntegrationCommand {
        command_id: IntegrationCommandId::new("cmd-1").unwrap(),
        connector_instance: ConnectorInstanceId::new("provider-prod-1").unwrap(),
        system: ExternalSystemId::new("provider-x").unwrap(),
        operation_kind: ExternalOperationKind::new("create-transfer").unwrap(),
        target: Some(ExternalObjectRef {
            system: ExternalSystemId::new("provider-x").unwrap(),
            object_type: ExternalObjectType::new("account").unwrap(),
            external_id: ExternalOpaqueId::new("acct-1").unwrap(),
        }),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(IdempotencyKey::new("idem-cmd-1").unwrap()),
        semantic_profile: SemanticProfileId::new("transfer@1").unwrap(),
        payload: DemoPayload(5000),
    }
}

fn profile() -> ProviderExecutionProfile {
    ProviderExecutionProfile {
        protocol_version: PROFILE_PROTOCOL_V1.to_owned(),
        profile_id: SemanticProfileId::new("provider-x-transfer@1").unwrap(),
        generation: 3,
        signer_key_id: "root-1".to_owned(),
        connector_instance: ConnectorInstanceId::new("provider-prod-1").unwrap(),
        system: ExternalSystemId::new("provider-x").unwrap(),
        operation_kind: ExternalOperationKind::new("create-transfer").unwrap(),
        required_target_type: Some(ExternalObjectType::new("account").unwrap()),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency: ProviderIdempotencySemantics::RequestKey {
            retention_ms: 86_400_000,
            lookup_supported: true,
            duplicate_returns_original: true,
        },
        reconciliation: ProviderReconciliationMode::IdempotencyKey,
        materializer_release: ContentCommitment::sha256(b"adapter-release-v1"),
        max_payload_bytes: 64 * 1024,
        not_before_ms: 100,
        not_after_ms: 1000,
    }
}

#[test]
fn canonical_v1_vectors_are_frozen() {
    let signing = SigningKey::from_bytes(&[7_u8; 32]);
    let root = ProviderProfileTrustRoot::new(
        "root-1",
        signing.verifying_key().to_bytes(),
        3,
    )
    .unwrap();
    let command = command();
    let profile = profile();
    let signature = signing.sign(&profile.canonical_preimage_v1().unwrap()).to_bytes();
    let qualified_profile = qualify_provider_profile(
        SignedProviderExecutionProfile { profile, signature },
        &root,
        200,
    )
    .unwrap();
    let claim = ExecutionClaim {
        entry_id: 7,
        attempt_id: ExecutionAttemptId::new("7:1").unwrap(),
        command_id: command.command_id.clone(),
        connector_instance: command.connector_instance.clone(),
        command_commitment: command.canonical_commitment_v1(),
        side_effect_class: command.side_effect_class,
        idempotency_key: command.idempotency_key.clone(),
        attempt_count: 1,
        lease_until_ms: 900,
    };
    let binding = qualify_execution_binding(&claim, &command, &qualified_profile, &root, 300)
        .unwrap();

    assert_eq!(
        command.canonical_commitment_v1().digest,
        [
            0x14, 0x36, 0x4b, 0x67, 0x94, 0xd2, 0xfd, 0xeb,
            0xe0, 0xc2, 0x77, 0x8e, 0x3c, 0x0e, 0x82, 0xae,
            0x7d, 0xf2, 0xcc, 0x6e, 0x51, 0x40, 0x16, 0xa8,
            0xa4, 0x46, 0xc1, 0xad, 0x23, 0x16, 0xfd, 0xb9,
        ]
    );
    assert_eq!(
        qualified_profile.profile_commitment().digest,
        [
            0x80, 0xe3, 0xcd, 0x70, 0x7d, 0xe2, 0xb0, 0x05,
            0xf0, 0xb4, 0x06, 0xd6, 0xff, 0x90, 0x7a, 0x2d,
            0x13, 0xdf, 0x01, 0x49, 0x05, 0x01, 0x74, 0x3d,
            0xff, 0xd2, 0x14, 0xd7, 0x2f, 0x2b, 0x21, 0x2c,
        ]
    );
    assert_eq!(
        root.root_commitment().digest,
        [
            0xfd, 0x74, 0x90, 0x2e, 0xeb, 0x3d, 0x7a, 0x14,
            0x65, 0x20, 0x1c, 0xbb, 0xb9, 0xe0, 0xb1, 0x56,
            0x4d, 0x87, 0xa4, 0x2a, 0x3c, 0xc9, 0xde, 0x43,
            0x76, 0x2b, 0x38, 0xc8, 0xc0, 0x00, 0xc7, 0x63,
        ]
    );
    assert_eq!(
        binding.binding_commitment().digest,
        [
            0xb2, 0x27, 0x32, 0x8a, 0x2d, 0xeb, 0x95, 0x47,
            0x3a, 0x2c, 0x42, 0x8c, 0x8e, 0x43, 0x89, 0x9c,
            0x00, 0x2e, 0xc8, 0x66, 0xbf, 0x6c, 0x10, 0xa5,
            0x8b, 0x78, 0x7a, 0x07, 0x0b, 0xc5, 0x87, 0x1e,
        ]
    );
}
