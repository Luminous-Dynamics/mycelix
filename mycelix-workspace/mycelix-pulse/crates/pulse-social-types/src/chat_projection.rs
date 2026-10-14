// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pure, transport-neutral projection of a qualified Pulse V2 durable message
//! into the social semantic contract.
//!
//! No Holochain types or storage access live here. A caller must obtain input
//! through a trusted V2 qualification boundary. The type system records the
//! boundary's required completeness metadata, but this crate cannot independently
//! prove that a remote zome call actually performed validation.

use serde::{Deserialize, Serialize};

use crate::{
    SocialContractError, SocialObjectIdV1, SocialObjectKind, SocialObjectRefV1,
    SocialProvenance, SocialRevisionV1, SOCIAL_SCHEMA_VERSION_V1,
};

pub const CHAT_PROJECTION_SCHEMA_VERSION_V1: u8 = 1;
pub const DURABLE_EVIDENCE_ID_MAX_BYTES: usize = 256;
pub const AGENT_KEY_RAW_BYTES: usize = 39;

/// Opaque identity of the durable Holochain evidence record.
///
/// This is intentionally not parsed as an ActionHash. The projection preserves
/// the exact qualified-boundary representation and makes no independent proof
/// claim about it.
#[derive(Clone, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub struct DurableEvidenceIdV1(pub String);

impl DurableEvidenceIdV1 {
    pub fn validate(&self) -> Result<(), ChatProjectionError> {
        if self.0.trim().is_empty() {
            return Err(ChatProjectionError::EmptyField("evidence_id"));
        }
        if self.0.len() > DURABLE_EVIDENCE_ID_MAX_BYTES {
            return Err(ChatProjectionError::FieldTooLong("evidence_id"));
        }
        Ok(())
    }
}

/// Raw agent identity carried by the qualified V2 envelope.
#[derive(Clone, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub struct AgentKeyV1(pub Vec<u8>);

impl AgentKeyV1 {
    pub fn validate(&self, field: &'static str) -> Result<(), ChatProjectionError> {
        if self.0.len() != AGENT_KEY_RAW_BYTES {
            return Err(ChatProjectionError::InvalidAgentKey(field));
        }
        Ok(())
    }
}

/// Completeness state supplied by the qualified V2 retrieval boundary.
///
/// The current get_inbox_v2 implementation drops several retrieval failures
/// instead of reporting them. Such a response must therefore be treated as
/// incomplete until a qualified boundary can account for those cases.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum V2InboxCompletenessV1 {
    Complete,
    Incomplete {
        skipped_links: u32,
        missing_records: u32,
        invalid_entries: u32,
    },
}

impl V2InboxCompletenessV1 {
    pub fn validate(&self) -> Result<(), ChatProjectionError> {
        match self {
            Self::Complete => Ok(()),
            Self::Incomplete {
                skipped_links,
                missing_records,
                invalid_entries,
            } => {
                if *skipped_links == 0 && *missing_records == 0 && *invalid_entries == 0 {
                    return Err(ChatProjectionError::InconsistentCompleteness);
                }
                Err(ChatProjectionError::IncompleteInbox)
            }
        }
    }
}

/// One V2 message after it has crossed the qualified retrieval boundary.
///
/// No verified boolean exists on purpose. Qualification is a boundary
/// property; the projection must not manufacture a cryptographic receipt from
/// ordinary wire data.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualifiedV2MessageV1 {
    pub message_id: [u8; 32],
    pub evidence_id: DurableEvidenceIdV1,
    pub sender: AgentKeyV1,
    pub recipient: AgentKeyV1,
    pub thread_id: Option<[u8; 32]>,
    pub in_reply_to: Option<[u8; 32]>,
    pub created_at_micros: i64,
}

impl QualifiedV2MessageV1 {
    pub fn validate(&self) -> Result<(), ChatProjectionError> {
        self.evidence_id.validate()?;
        self.sender.validate("sender")?;
        self.recipient.validate("recipient")?;
        if self.created_at_micros < 0 {
            return Err(ChatProjectionError::InvalidTimestamp);
        }
        normalize_thread_relation(self.message_id, self.thread_id, self.in_reply_to)?;
        Ok(())
    }
}

/// Explicit Chat relationship semantics derived only from authenticated V2
/// thread/reply metadata. No root relationship is inferred when only a thread
/// identifier is present.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum ChatThreadRelationV1 {
    Unthreaded,
    Threaded {
        thread_id: [u8; 32],
        in_reply_to: Option<[u8; 32]>,
    },
}

pub fn normalize_thread_relation(
    message_id: [u8; 32],
    thread_id: Option<[u8; 32]>,
    in_reply_to: Option<[u8; 32]>,
) -> Result<ChatThreadRelationV1, ChatProjectionError> {
    match (thread_id, in_reply_to) {
        (None, None) => Ok(ChatThreadRelationV1::Unthreaded),
        (Some(thread_id), None) => {
            if thread_id == [0; 32] {
                return Err(ChatProjectionError::ZeroThreadId);
            }
            Ok(ChatThreadRelationV1::Threaded { thread_id, in_reply_to: None })
        }
        (Some(thread_id), Some(reply)) => {
            if thread_id == [0; 32] {
                return Err(ChatProjectionError::ZeroThreadId);
            }
            if reply == [0; 32] {
                return Err(ChatProjectionError::ZeroReplyId);
            }
            if reply == message_id {
                return Err(ChatProjectionError::SelfReply);
            }
            Ok(ChatThreadRelationV1::Threaded {
                thread_id,
                in_reply_to: Some(reply),
            })
        }
        (None, Some(_)) => Err(ChatProjectionError::ReplyWithoutThread),
    }
}

/// Semantic Chat projection with no mutable UI state and no second durable
/// store identity.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChatMessageProjectionV1 {
    pub schema_version: u8,
    pub object: SocialObjectRefV1,
    pub evidence_id: DurableEvidenceIdV1,
    pub sender: AgentKeyV1,
    pub recipient: AgentKeyV1,
    pub thread: ChatThreadRelationV1,
    pub created_at_micros: i64,
}

impl ChatMessageProjectionV1 {
    pub fn validate(&self) -> Result<(), ChatProjectionError> {
        if self.schema_version != CHAT_PROJECTION_SCHEMA_VERSION_V1 {
            return Err(ChatProjectionError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        self.object
            .validate()
            .map_err(ChatProjectionError::SocialContract)?;
        if self.object.kind != SocialObjectKind::ChatMessage {
            return Err(ChatProjectionError::WrongObjectKind);
        }
        self.evidence_id.validate()?;
        self.sender.validate("sender")?;
        self.recipient.validate("recipient")?;
        if self.created_at_micros < 0 {
            return Err(ChatProjectionError::InvalidTimestamp);
        }
        Ok(())
    }
}

/// Deterministic projection of one qualified durable V2 record.
///
/// The durable evidence identity remains opaque. The logical message ID is
/// encoded separately, so a replay can be recognized by evidence identity
/// without conflating it with semantic message identity.
pub fn project_v2_message(
    input: &QualifiedV2MessageV1,
) -> Result<ChatMessageProjectionV1, ChatProjectionError> {
    input.validate()?;

    let logical_id = SocialObjectIdV1(format!(
        "pulse-v2:message:{}",
        encode_hex(&input.message_id)
    ));
    let object = SocialObjectRefV1 {
        schema_version: SOCIAL_SCHEMA_VERSION_V1,
        object_id: logical_id,
        revision: SocialRevisionV1(input.evidence_id.0.clone()),
        kind: SocialObjectKind::ChatMessage,
        provenance: SocialProvenance::MycelixCanonical,
    };
    let thread = normalize_thread_relation(input.message_id, input.thread_id, input.in_reply_to)?;

    let projection = ChatMessageProjectionV1 {
        schema_version: CHAT_PROJECTION_SCHEMA_VERSION_V1,
        object,
        evidence_id: input.evidence_id.clone(),
        sender: input.sender.clone(),
        recipient: input.recipient.clone(),
        thread,
        created_at_micros: input.created_at_micros,
    };
    projection.validate()?;
    Ok(projection)
}

/// Project a qualified inbox only when the retrieval boundary can explicitly
/// establish that no records were silently discarded.
///
/// Ordering is timestamp descending, then opaque evidence identity ascending.
/// The second key makes ties deterministic without treating client time as
/// global chronology.
pub fn project_v2_inbox(
    completeness: &V2InboxCompletenessV1,
    inputs: &[QualifiedV2MessageV1],
) -> Result<Vec<ChatMessageProjectionV1>, ChatProjectionError> {
    completeness.validate()?;

    let mut projections = inputs
        .iter()
        .map(project_v2_message)
        .collect::<Result<Vec<_>, _>>()?;

    projections.sort_by(|a, b| {
        b.created_at_micros
            .cmp(&a.created_at_micros)
            .then_with(|| a.evidence_id.0.cmp(&b.evidence_id.0))
    });

    Ok(projections)
}

/// Duplicate/replay identity is durable evidence identity, not logical message
/// identity. This helper intentionally does not decide whether a UI should
/// collapse messages with the same logical ID.
pub fn same_durable_evidence(
    left: &ChatMessageProjectionV1,
    right: &ChatMessageProjectionV1,
) -> bool {
    left.evidence_id == right.evidence_id
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ChatProjectionError {
    UnsupportedSchemaVersion(u8),
    EmptyField(&'static str),
    FieldTooLong(&'static str),
    InvalidAgentKey(&'static str),
    InvalidTimestamp,
    IncompleteInbox,
    InconsistentCompleteness,
    ReplyWithoutThread,
    ZeroThreadId,
    ZeroReplyId,
    SelfReply,
    WrongObjectKind,
    SocialContract(SocialContractError),
}

fn encode_hex(bytes: &[u8; 32]) -> String {
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

    fn key(byte: u8) -> AgentKeyV1 {
        AgentKeyV1(vec![byte; AGENT_KEY_RAW_BYTES])
    }

    fn message(byte: u8) -> QualifiedV2MessageV1 {
        QualifiedV2MessageV1 {
            message_id: [byte; 32],
            evidence_id: DurableEvidenceIdV1(format!("uhCAk{byte:02x}")),
            sender: key(1),
            recipient: key(2),
            thread_id: None,
            in_reply_to: None,
            created_at_micros: 10,
        }
    }

    #[test]
    fn logical_id_is_distinct_from_durable_evidence_identity() {
        let projection = project_v2_message(&message(1)).unwrap();
        assert_eq!(
            projection.object.object_id.0,
            "pulse-v2:message:0101010101010101010101010101010101010101010101010101010101010101"
        );
        assert_eq!(projection.object.revision.0, projection.evidence_id.0);
        assert_ne!(projection.object.object_id.0, projection.evidence_id.0);
    }

    #[test]
    fn reply_requires_thread() {
        let mut input = message(1);
        input.in_reply_to = Some([2; 32]);
        assert_eq!(
            project_v2_message(&input),
            Err(ChatProjectionError::ReplyWithoutThread)
        );
    }

    #[test]
    fn inconsistent_thread_reply_pair_fails_closed() {
        let mut input = message(1);
        input.thread_id = Some([3; 32]);
        input.in_reply_to = Some([1; 32]);
        assert_eq!(
            project_v2_message(&input),
            Err(ChatProjectionError::SelfReply)
        );
    }

    #[test]
    fn absent_metadata_is_not_invented_as_a_thread() {
        assert_eq!(
            normalize_thread_relation([1; 32], None, None).unwrap(),
            ChatThreadRelationV1::Unthreaded
        );
    }

    #[test]
    fn explicit_thread_without_reply_is_preserved() {
        assert_eq!(
            normalize_thread_relation([1; 32], Some([7; 32]), None).unwrap(),
            ChatThreadRelationV1::Threaded {
                thread_id: [7; 32],
                in_reply_to: None,
            }
        );
    }

    #[test]
    fn duplicate_evidence_is_detectable_without_collapsing_logical_ids() {
        let input = message(1);
        let a = project_v2_message(&input).unwrap();
        let b = project_v2_message(&input).unwrap();
        assert!(same_durable_evidence(&a, &b));
        assert_eq!(a.object.object_id, b.object.object_id);
    }

    #[test]
    fn same_logical_id_can_have_distinct_durable_evidence() {
        let input = message(1);
        let mut other = input.clone();
        other.evidence_id = DurableEvidenceIdV1("different-action-hash".into());
        let a = project_v2_message(&input).unwrap();
        let b = project_v2_message(&other).unwrap();
        assert_eq!(a.object.object_id, b.object.object_id);
        assert_ne!(a.evidence_id, b.evidence_id);
    }

    #[test]
    fn incomplete_retrieval_fails_closed() {
        let completeness = V2InboxCompletenessV1::Incomplete {
            skipped_links: 1,
            missing_records: 0,
            invalid_entries: 0,
        };
        assert_eq!(
            project_v2_inbox(&completeness, &[message(1)]),
            Err(ChatProjectionError::IncompleteInbox)
        );
    }

    #[test]
    fn zero_thread_and_reply_ids_fail_closed() {
        let mut root = message(1);
        root.thread_id = Some([0; 32]);
        assert_eq!(
            project_v2_message(&root),
            Err(ChatProjectionError::ZeroThreadId)
        );

        let mut reply = message(1);
        reply.thread_id = Some([4; 32]);
        reply.in_reply_to = Some([0; 32]);
        assert_eq!(
            project_v2_message(&reply),
            Err(ChatProjectionError::ZeroReplyId)
        );
    }

    #[test]
    fn ordering_uses_timestamp_then_evidence_identity() {
        let mut a = message(1);
        let mut b = message(2);
        a.created_at_micros = 50;
        b.created_at_micros = 50;
        a.evidence_id = DurableEvidenceIdV1("b".into());
        b.evidence_id = DurableEvidenceIdV1("a".into());

        let out = project_v2_inbox(&V2InboxCompletenessV1::Complete, &[a, b]).unwrap();
        assert_eq!(out[0].evidence_id.0, "a");
        assert_eq!(out[1].evidence_id.0, "b");
    }

    #[test]
    fn invalid_agent_key_fails_closed() {
        let mut input = message(1);
        input.sender = AgentKeyV1(vec![1; 38]);
        assert_eq!(
            project_v2_message(&input),
            Err(ChatProjectionError::InvalidAgentKey("sender"))
        );
    }
    #[test]
    fn projection_serde_round_trip_preserves_evidence_and_thread() {
        let mut input = message(1);
        input.thread_id = Some([7; 32]);
        input.in_reply_to = Some([2; 32]);
        let projection = project_v2_message(&input).unwrap();
        let encoded = serde_json::to_string(&projection).unwrap();
        let decoded = serde_json::from_str::<ChatMessageProjectionV1>(&encoded).unwrap();
        assert_eq!(decoded, projection);
    }

    #[test]
    fn projection_serde_rejects_unknown_fields() {
        let input = message(1);
        let projection = project_v2_message(&input).unwrap();
        let mut encoded = serde_json::to_value(&projection).unwrap();
        encoded.as_object_mut().unwrap().insert(
            "authority".into(),
            serde_json::Value::String("admin".into()),
        );
        assert!(serde_json::from_value::<ChatMessageProjectionV1>(encoded).is_err());
    }

}
