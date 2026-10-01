
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Deterministic projection contracts for durable Pulse V2 messages.
//!
//! This module is deliberately transport-neutral. A Holochain adapter must first
//! establish that the supplied V2 record is authenticated, durably committed,
//! and still valid. Realtime signals are not accepted as inputs here.

const ACTION_HASH_BYTES: usize = 39;
const AGENT_PUBKEY_BYTES: usize = 39;
const MESSAGE_ID_BYTES: usize = 32;
const MAX_CIPHERTEXT_BYTES: usize = 2 * 1024 * 1024;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerifiedV2MessageV1 {
    pub message_id: [u8; MESSAGE_ID_BYTES],
    pub action_hash: [u8; ACTION_HASH_BYTES],
    pub sender: Vec<u8>,
    pub recipient: Vec<u8>,
    pub thread_id: Option<[u8; MESSAGE_ID_BYTES]>,
    pub in_reply_to: Option<[u8; MESSAGE_ID_BYTES]>,
    pub created_at_micros: i64,
    pub durable_committed_at_micros: i64,
    pub ciphertext_len: usize,
    pub agent_signature_verified: bool,
    pub ml_dsa_signature_verified: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ChatThreadRelationV1 {
    Unspecified,
    ThreadRootOrContext,
    Reply,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ChatEvidenceV1 {
    pub durable: bool,
    pub agent_signature_verified: bool,
    pub ml_dsa_signature_verified: bool,
    pub ciphertext_present: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ChatOrderingKeyV1 {
    pub durable_committed_at_micros: i64,
    pub created_at_micros: i64,
    pub message_id: [u8; MESSAGE_ID_BYTES],
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ChatMessageProjectionV1 {
    pub object: SocialObjectRefV1,
    pub sender: Vec<u8>,
    pub recipient: Vec<u8>,
    pub thread_id: Option<[u8; MESSAGE_ID_BYTES]>,
    pub in_reply_to: Option<[u8; MESSAGE_ID_BYTES]>,
    pub thread_relation: ChatThreadRelationV1,
    pub evidence: ChatEvidenceV1,
    pub ordering: ChatOrderingKeyV1,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ChatTombstoneV1 {
    pub object: SocialObjectRefV1,
    pub deletion_action_hash: [u8; ACTION_HASH_BYTES],
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ChatProjectionError {
    InvalidActionHash,
    InvalidSender,
    InvalidRecipient,
    InvalidTimestamp,
    InvalidCiphertextLength,
    MissingVerificationEvidence,
    InvalidReplyWithoutThread,
}

impl VerifiedV2MessageV1 {
    pub fn validate(&self) -> Result<(), ChatProjectionError> {
        if self.action_hash.len() != ACTION_HASH_BYTES {
            return Err(ChatProjectionError::InvalidActionHash);
        }
        if self.sender.len() != AGENT_PUBKEY_BYTES {
            return Err(ChatProjectionError::InvalidSender);
        }
        if self.recipient.len() != AGENT_PUBKEY_BYTES {
            return Err(ChatProjectionError::InvalidRecipient);
        }
        if self.created_at_micros < 0 || self.durable_committed_at_micros < 0 {
            return Err(ChatProjectionError::InvalidTimestamp);
        }
        if self.ciphertext_len > MAX_CIPHERTEXT_BYTES {
            return Err(ChatProjectionError::InvalidCiphertextLength);
        }
        if !self.agent_signature_verified || !self.ml_dsa_signature_verified {
            return Err(ChatProjectionError::MissingVerificationEvidence);
        }
        if self.in_reply_to.is_some() && self.thread_id.is_none() {
            return Err(ChatProjectionError::InvalidReplyWithoutThread);
        }
        Ok(())
    }
}

pub fn classify_thread_relation(
    thread_id: Option<[u8; MESSAGE_ID_BYTES]>,
    in_reply_to: Option<[u8; MESSAGE_ID_BYTES]>,
) -> Result<ChatThreadRelationV1, ChatProjectionError> {
    match (thread_id, in_reply_to) {
        (None, None) => Ok(ChatThreadRelationV1::Unspecified),
        (Some(_), None) => Ok(ChatThreadRelationV1::ThreadRootOrContext),
        (Some(_), Some(_)) => Ok(ChatThreadRelationV1::Reply),
        (None, Some(_)) => Err(ChatProjectionError::InvalidReplyWithoutThread),
    }
}

pub fn project_chat_message_v1(
    message: &VerifiedV2MessageV1,
) -> Result<ChatMessageProjectionV1, ChatProjectionError> {
    message.validate()?;
    let thread_relation = classify_thread_relation(message.thread_id, message.in_reply_to)?;

    let object = SocialObjectRefV1 {
        schema_version: SOCIAL_SCHEMA_VERSION_V1,
        object_id: SocialObjectIdV1(format!("v2:msg:{}", hex(&message.message_id))),
        revision: SocialRevisionV1(format!(
            "holochain:action:{}",
            hex(&message.action_hash)
        )),
        kind: SocialObjectKind::ChatMessage,
        provenance: SocialProvenance::MycelixCanonical,
    };

    object
        .validate()
        .map_err(|_| ChatProjectionError::InvalidActionHash)?;

    Ok(ChatMessageProjectionV1 {
        object,
        sender: message.sender.clone(),
        recipient: message.recipient.clone(),
        thread_id: message.thread_id,
        in_reply_to: message.in_reply_to,
        thread_relation,
        evidence: ChatEvidenceV1 {
            durable: true,
            agent_signature_verified: message.agent_signature_verified,
            ml_dsa_signature_verified: message.ml_dsa_signature_verified,
            ciphertext_present: message.ciphertext_len > 0,
        },
        ordering: ChatOrderingKeyV1 {
            durable_committed_at_micros: message.durable_committed_at_micros,
            created_at_micros: message.created_at_micros,
            message_id: message.message_id,
        },
    })
}

pub fn project_chat_tombstone_v1(
    message: &ChatMessageProjectionV1,
    deletion_action_hash: [u8; ACTION_HASH_BYTES],
) -> ChatTombstoneV1 {
    ChatTombstoneV1 {
        object: message.object.clone(),
        deletion_action_hash,
    }
}

fn hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    fn message() -> VerifiedV2MessageV1 {
        VerifiedV2MessageV1 {
            message_id: [0x11; MESSAGE_ID_BYTES],
            action_hash: [0x22; ACTION_HASH_BYTES],
            sender: vec![0x33; AGENT_PUBKEY_BYTES],
            recipient: vec![0x44; AGENT_PUBKEY_BYTES],
            thread_id: Some([0x55; MESSAGE_ID_BYTES]),
            in_reply_to: Some([0x66; MESSAGE_ID_BYTES]),
            created_at_micros: 20,
            durable_committed_at_micros: 30,
            ciphertext_len: 128,
            agent_signature_verified: true,
            ml_dsa_signature_verified: true,
        }
    }

    #[test]
    fn verified_v2_message_projects_deterministically() {
        let first = project_chat_message_v1(&message()).unwrap();
        let second = project_chat_message_v1(&message()).unwrap();
        assert_eq!(first, second);
        assert_eq!(
            first.object.object_id.0,
            "v2:msg:1111111111111111111111111111111111111111111111111111111111111111"
        );
        assert_eq!(first.thread_relation, ChatThreadRelationV1::Reply);
        assert!(first.evidence.durable);
    }

    #[test]
    fn action_hash_is_revision_not_object_identity() {
        let mut second = message();
        second.action_hash = [0x77; ACTION_HASH_BYTES];

        let first = project_chat_message_v1(&message()).unwrap();
        let second = project_chat_message_v1(&second).unwrap();

        assert_eq!(first.object.object_id, second.object.object_id);
        assert_ne!(first.object.revision, second.object.revision);
    }

    #[test]
    fn thread_context_does_not_infer_root() {
        assert_eq!(
            classify_thread_relation(Some([1; MESSAGE_ID_BYTES]), None).unwrap(),
            ChatThreadRelationV1::ThreadRootOrContext
        );
    }

    #[test]
    fn reply_without_thread_fails_closed() {
        assert_eq!(
            classify_thread_relation(None, Some([1; MESSAGE_ID_BYTES])),
            Err(ChatProjectionError::InvalidReplyWithoutThread)
        );
    }

    #[test]
    fn unsigned_record_fails_closed() {
        let mut input = message();
        input.agent_signature_verified = false;
        assert_eq!(
            project_chat_message_v1(&input),
            Err(ChatProjectionError::MissingVerificationEvidence)
        );
    }

    #[test]
    fn invalid_agent_key_lengths_fail_closed() {
        let mut input = message();
        input.sender.pop();
        assert_eq!(
            project_chat_message_v1(&input),
            Err(ChatProjectionError::InvalidSender)
        );
    }

    #[test]
    fn negative_timestamps_fail_closed() {
        let mut input = message();
        input.created_at_micros = -1;
        assert_eq!(
            project_chat_message_v1(&input),
            Err(ChatProjectionError::InvalidTimestamp)
        );
    }

    #[test]
    fn oversized_ciphertext_fails_closed() {
        let mut input = message();
        input.ciphertext_len = MAX_CIPHERTEXT_BYTES + 1;
        assert_eq!(
            project_chat_message_v1(&input),
            Err(ChatProjectionError::InvalidCiphertextLength)
        );
    }

    #[test]
    fn ordering_uses_durable_time_before_client_time() {
        let mut earlier = message();
        earlier.durable_committed_at_micros = 10;
        earlier.created_at_micros = 900;

        let mut later = message();
        later.durable_committed_at_micros = 20;
        later.created_at_micros = 1;

        let a = project_chat_message_v1(&earlier).unwrap();
        let b = project_chat_message_v1(&later).unwrap();
        assert!(a.ordering < b.ordering);
    }

    #[test]
    fn tombstone_preserves_object_identity() {
        let projection = project_chat_message_v1(&message()).unwrap();
        let tombstone = project_chat_tombstone_v1(&projection, [0x99; ACTION_HASH_BYTES]);
        assert_eq!(tombstone.object.object_id, projection.object.object_id);
        assert_eq!(tombstone.object.revision, projection.object.revision);
        assert_eq!(tombstone.deletion_action_hash, [0x99; ACTION_HASH_BYTES]);
    }
}
