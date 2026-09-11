use mycelix_governance_authority::ProposalId;
use mycelix_integration_authority_action::qualify_integration_authority_action;
use mycelix_integration_core::{
    CanonicalEncodeV1, ConnectorInstanceId, ExternalObjectRef, ExternalObjectType,
    ExternalOpaqueId, ExternalOperationKind, ExternalSystemId, IdempotencyKey,
    IntegrationCommand, IntegrationCommandId, SemanticProfileId, SideEffectClass,
};

#[derive(Debug, Clone, PartialEq, Eq)]
struct DemoPayload(u64);

impl CanonicalEncodeV1 for DemoPayload {
    fn canonical_preimage_v1(&self) -> Vec<u8> {
        let mut out = b"DEMO-PAYLOAD\0V1\0".to_vec();
        out.extend_from_slice(&self.0.to_be_bytes());
        out
    }
}

#[test]
fn exact_authority_action_json_v1_is_frozen() {
    let command = IntegrationCommand {
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
    };
    let action = qualify_integration_authority_action(
        &ProposalId::new("proposal:transfer:1").unwrap(),
        &command,
    )
    .unwrap();

    assert_eq!(
        action.exact_action_json(),
        concat!(
            "{\"protocol_version\":\"mycelix-integration-authority-action-v1\",",
            "\"action_type\":\"integration_command\",",
            "\"proposal_id\":\"proposal:transfer:1\",",
            "\"command_encoding\":\"mycelix-integration-command-canonical-v1-sha256\",",
            "\"command_id\":\"cmd-1\",",
            "\"command_commitment_sha256\":\"14364b6794d2fdebe0c2778e3c0e82ae7df2cc6e514016a8a446c1ad2316fdb9\",",
            "\"connector_instance\":\"provider-prod-1\",",
            "\"system\":\"provider-x\",",
            "\"operation_kind\":\"create-transfer\",",
            "\"target\":{\"object_type\":\"account\",\"external_id\":\"acct-1\"},",
            "\"side_effect_class\":\"irreversible\",",
            "\"idempotency_key\":\"idem-cmd-1\",",
            "\"semantic_profile\":\"transfer@1\"}"
        )
    );
}
