//! Deterministic exact-byte governance action bridge for Mycelix integration commands.
//!
//! This crate closes one translation theorem and no more:
//!
//! ```text
//! ProposalId + exact typed IntegrationCommand
//!     -> exact compact JSON governance action bytes
//!     -> mycelix-execution-action-digest
//! ```
//!
//! The generated action contains the SHA-256 commitment of the command's
//! `CanonicalEncodeV1` bytes plus its security-relevant routing metadata. A
//! governance threshold/designation can therefore authorize this exact command
//! without this crate becoming a provider-payload materializer or authority
//! source.

use mycelix_execution_action_digest::{
    execution_authority_digest, ActionDigestError, ACTIONS_DIGEST_PROFILE_V1,
};
use mycelix_governance_authority::ProposalId;
use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{
    CanonicalEncodeV1, ContentCommitment, IntegrationCommand, SideEffectClass,
};
use serde::Serialize;
use thiserror::Error;

pub const INTEGRATION_AUTHORITY_ACTION_PROTOCOL_V1: &str =
    "mycelix-integration-authority-action-v1";
pub const INTEGRATION_COMMAND_COMMITMENT_PROFILE_V1: &str =
    "mycelix-integration-command-canonical-v1-sha256";

#[derive(Debug, Clone)]
pub struct QualifiedIntegrationAuthorityAction {
    proposal_id: ProposalId,
    exact_action_json: String,
    actions_digest: Digest32,
    command_commitment: ContentCommitment,
}

impl QualifiedIntegrationAuthorityAction {
    pub fn proposal_id(&self) -> &ProposalId {
        &self.proposal_id
    }

    pub fn exact_action_json(&self) -> &str {
        &self.exact_action_json
    }

    pub fn actions_digest(&self) -> Digest32 {
        self.actions_digest
    }

    pub fn actions_digest_profile(&self) -> &'static str {
        ACTIONS_DIGEST_PROFILE_V1
    }

    pub fn command_commitment(&self) -> &ContentCommitment {
        &self.command_commitment
    }

    pub const fn exact_command_bound_here(&self) -> bool {
        true
    }

    pub const fn governance_digest_computed_here(&self) -> bool {
        true
    }

    pub const fn authority_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn current_authority_verified_here(&self) -> bool {
        false
    }

    pub const fn capability_semantics_closed_here(&self) -> bool {
        false
    }

    pub const fn provider_payload_materialized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Serialize)]
struct AuthorityActionWire<'a> {
    protocol_version: &'static str,
    action_type: &'static str,
    proposal_id: &'a str,
    command_encoding: &'static str,
    command_id: &'a str,
    command_commitment_sha256: String,
    connector_instance: &'a str,
    system: &'a str,
    operation_kind: &'a str,
    target: Option<TargetWire<'a>>,
    side_effect_class: &'static str,
    idempotency_key: Option<&'a str>,
    semantic_profile: &'a str,
}

#[derive(Serialize)]
struct TargetWire<'a> {
    object_type: &'a str,
    external_id: &'a str,
}

pub fn qualify_integration_authority_action<C>(
    proposal_id: &ProposalId,
    command: &IntegrationCommand<C>,
) -> Result<QualifiedIntegrationAuthorityAction, IntegrationAuthorityActionError>
where
    C: CanonicalEncodeV1,
{
    if let Some(target) = &command.target
        && target.system != command.system
    {
        return Err(IntegrationAuthorityActionError::TargetSystemMismatch);
    }

    let command_commitment = command.canonical_commitment_v1();
    let wire = AuthorityActionWire {
        protocol_version: INTEGRATION_AUTHORITY_ACTION_PROTOCOL_V1,
        action_type: "integration_command",
        proposal_id: proposal_id.as_str(),
        command_encoding: INTEGRATION_COMMAND_COMMITMENT_PROFILE_V1,
        command_id: command.command_id.as_str(),
        command_commitment_sha256: lower_hex(&command_commitment.digest),
        connector_instance: command.connector_instance.as_str(),
        system: command.system.as_str(),
        operation_kind: command.operation_kind.as_str(),
        target: command.target.as_ref().map(|target| TargetWire {
            object_type: target.object_type.as_str(),
            external_id: target.external_id.as_str(),
        }),
        side_effect_class: side_effect_name(command.side_effect_class),
        idempotency_key: command.idempotency_key.as_ref().map(|value| value.as_str()),
        semantic_profile: command.semantic_profile.as_str(),
    };
    let exact_action_json = serde_json::to_string(&wire)?;
    let actions_digest = execution_authority_digest(proposal_id.as_str(), &exact_action_json)?;

    Ok(QualifiedIntegrationAuthorityAction {
        proposal_id: proposal_id.clone(),
        exact_action_json,
        actions_digest,
        command_commitment,
    })
}

#[derive(Debug, Error)]
pub enum IntegrationAuthorityActionError {
    #[error("integration command target belongs to a different external system")]
    TargetSystemMismatch,
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    ActionDigest(#[from] ActionDigestError),
}

fn side_effect_name(value: SideEffectClass) -> &'static str {
    match value {
        SideEffectClass::ReadOnly => "read_only",
        SideEffectClass::Reversible => "reversible",
        SideEffectClass::Compensatable => "compensatable",
        SideEffectClass::Irreversible => "irreversible",
    }
}

fn lower_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_integration_core::{
        ConnectorInstanceId, ExternalObjectRef, ExternalObjectType, ExternalOpaqueId,
        ExternalOperationKind, ExternalSystemId, IdempotencyKey, IntegrationCommandId,
        SemanticProfileId,
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

    fn proposal(value: &str) -> ProposalId {
        ProposalId::new(value).unwrap()
    }

    fn command(amount: u64) -> IntegrationCommand<DemoPayload> {
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
            payload: DemoPayload(amount),
        }
    }

    #[test]
    fn exact_command_produces_stable_exact_action() {
        let first = qualify_integration_authority_action(
            &proposal("proposal:transfer:1"),
            &command(5000),
        )
        .unwrap();
        let second = qualify_integration_authority_action(
            &proposal("proposal:transfer:1"),
            &command(5000),
        )
        .unwrap();
        assert_eq!(first.exact_action_json(), second.exact_action_json());
        assert_eq!(first.actions_digest(), second.actions_digest());
        assert_eq!(first.actions_digest_profile(), ACTIONS_DIGEST_PROFILE_V1);
        assert!(first.exact_command_bound_here());
        assert!(!first.grants_execution_authority());
    }

    #[test]
    fn payload_mutation_changes_governance_action_identity() {
        let first = qualify_integration_authority_action(
            &proposal("proposal:transfer:1"),
            &command(5000),
        )
        .unwrap();
        let second = qualify_integration_authority_action(
            &proposal("proposal:transfer:1"),
            &command(5001),
        )
        .unwrap();
        assert_ne!(first.command_commitment(), second.command_commitment());
        assert_ne!(first.actions_digest(), second.actions_digest());
    }

    #[test]
    fn governance_proposal_identity_is_not_reusable() {
        let command = command(5000);
        let first = qualify_integration_authority_action(&proposal("proposal:transfer:1"), &command)
            .unwrap();
        let second = qualify_integration_authority_action(&proposal("proposal:transfer:2"), &command)
            .unwrap();
        assert_ne!(first.actions_digest(), second.actions_digest());
    }

    #[test]
    fn cross_system_target_fails_closed() {
        let mut command = command(5000);
        command.target.as_mut().unwrap().system = ExternalSystemId::new("provider-y").unwrap();
        assert!(matches!(
            qualify_integration_authority_action(&proposal("proposal:transfer:1"), &command),
            Err(IntegrationAuthorityActionError::TargetSystemMismatch)
        ));
    }

    #[test]
    fn translation_does_not_claim_capability_or_current_authority() {
        let value = qualify_integration_authority_action(
            &proposal("proposal:transfer:1"),
            &command(5000),
        )
        .unwrap();
        assert!(value.governance_digest_computed_here());
        assert!(!value.authority_origin_verified_here());
        assert!(!value.current_authority_verified_here());
        assert!(!value.capability_semantics_closed_here());
        assert!(!value.provider_payload_materialized_here());
        assert!(!value.grants_execution_authority());
    }
}
