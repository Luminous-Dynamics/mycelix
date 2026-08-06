// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/// Messaging Integrity Zome
///
/// This zome provides P2P encrypted messaging for the Mycelix-Marketplace.
/// Features:
/// - End-to-end encrypted messages
/// - Conversation threading (by listing or transaction)
/// - Rich media attachments (IPFS)
/// - Read receipts and typing indicators
/// - MATL-gated messaging (spam prevention)
/// - Message search and filtering
use hdi::prelude::*;

/// Message entry - represents a single message in a conversation
///
/// Messages are encrypted client-side before being stored on the DHT.
/// Only the sender and recipient can decrypt the content.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Message {
    /// Sender's public key
    pub sender: AgentPubKey,

    /// Recipient's public key
    pub recipient: AgentPubKey,

    /// Encrypted message content (AES-256-GCM)
    /// Structure when decrypted: {"text": "...", "attachments": [...]}
    pub encrypted_content: String,

    /// Optional: Link to listing this message relates to
    pub listing_hash: Option<ActionHash>,

    /// Optional: Link to transaction this message relates to
    pub transaction_hash: Option<ActionHash>,

    /// Conversation thread ID (hash of first message in thread)
    pub conversation_id: ActionHash,

    /// Message sent timestamp
    pub sent_at: u64,

    /// Message read timestamp (None if unread)
    pub read_at: Option<u64>,

    /// Message type for UI rendering
    pub message_type: MessageType,

    /// Epistemic classification
    pub epistemic: EpistemicClassification,
}

/// Message types for different UI contexts
#[derive(Clone, PartialEq, Serialize, Deserialize, Debug)]
pub enum MessageType {
    /// Regular text message
    Text,

    /// System message (transaction update, etc.)
    System,

    /// Offer message (price negotiation)
    Offer,

    /// Question about listing
    Question,

    /// Review request
    ReviewRequest,
}

/// Conversation metadata - summary of a message thread
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Conversation {
    /// Participants in conversation (usually 2)
    pub participants: Vec<AgentPubKey>,

    /// Related listing (if any)
    pub listing_hash: Option<ActionHash>,

    /// Related transaction (if any)
    pub transaction_hash: Option<ActionHash>,

    /// Conversation subject/title
    pub subject: String,

    /// First message hash (conversation ID)
    pub first_message_hash: ActionHash,

    /// Last message hash
    pub last_message_hash: ActionHash,

    /// Total message count
    pub message_count: u32,

    /// Unread count for each participant
    pub unread_counts: Vec<(AgentPubKey, u32)>,

    /// Conversation started timestamp
    pub started_at: u64,

    /// Last activity timestamp
    pub last_activity_at: u64,

    /// Conversation status
    pub status: ConversationStatus,
}

/// Conversation status
#[derive(Clone, PartialEq, Serialize, Deserialize, Debug)]
pub enum ConversationStatus {
    /// Active conversation
    Active,

    /// Archived by one or both participants
    Archived,

    /// Muted notifications
    Muted,

    /// Blocked (spam/abuse)
    Blocked,
}

/// Read receipt - tracks when messages are read
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ReadReceipt {
    /// Message that was read
    pub message_hash: ActionHash,

    /// Who read it
    pub reader: AgentPubKey,

    /// When it was read
    pub read_at: u64,
}

/// Typing indicator - ephemeral signal that user is typing
#[derive(Clone, PartialEq, Serialize, Deserialize, Debug)]
pub struct TypingIndicator {
    /// Conversation ID
    pub conversation_id: ActionHash,

    /// Who is typing
    pub typer: AgentPubKey,

    /// Timestamp (expires after 5 seconds)
    pub timestamp: u64,
}

/// Message attachment metadata (actual file on IPFS)
#[derive(Clone, PartialEq, Serialize, Deserialize, Debug)]
pub struct Attachment {
    /// IPFS CID
    pub ipfs_cid: String,

    /// Original filename
    pub filename: String,

    /// MIME type
    pub mime_type: String,

    /// File size in bytes
    pub size_bytes: u64,

    /// Optional thumbnail CID (for images/videos)
    pub thumbnail_cid: Option<String>,
}

/// Epistemic classification for messages
#[derive(Clone, PartialEq, Serialize, Deserialize, Debug)]
pub struct EpistemicClassification {
    /// Empirical level
    pub empirical: EmpiricalLevel,

    /// Normative level
    pub normative: NormativeLevel,

    /// Materiality level
    pub materiality: MaterialityLevel,
}

/// Empirical classification levels
#[derive(Clone, PartialEq, Serialize, Deserialize, Debug)]
pub enum EmpiricalLevel {
    /// E1: Testimonial (sender claims, recipient trusts)
    E1Testimonial,

    /// E2: Privately verifiable (both can verify)
    E2PrivateVerify,

    /// E3: Publicly verifiable (anyone can verify)
    E3PublicVerify,
}

/// Normative classification levels
#[derive(Clone, PartialEq, Serialize, Deserialize, Debug)]
pub enum NormativeLevel {
    /// N1: Communal (buyer-seller agreement)
    N1Communal,

    /// N2: Marketplace (whole marketplace standards)
    N2Marketplace,

    /// N3: Universal (global standards)
    N3Universal,
}

/// Materiality classification levels
#[derive(Clone, PartialEq, Serialize, Deserialize, Debug)]
pub enum MaterialityLevel {
    /// M1: Temporal (ephemeral messages)
    M1Temporal,

    /// M2: Persistent (archived conversations)
    M2Persistent,

    /// M3: Permanent (dispute evidence)
    M3Permanent,
}

/// Entry types for messaging zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Message(Message),
    Conversation(Conversation),
    ReadReceipt(ReadReceipt),
}

/// Link types for messaging relationships
#[hdk_link_types]
pub enum LinkTypes {
    /// Agent -> Conversations (as participant)
    AgentToConversations,

    /// Agent -> Sent Messages
    AgentToSentMessages,

    /// Agent -> Received Messages
    AgentToReceivedMessages,

    /// Conversation -> Messages
    ConversationToMessages,

    /// Message -> Read Receipts
    MessageToReadReceipts,

    /// Listing -> Conversations
    ListingToConversations,

    /// Transaction -> Conversations
    TransactionToConversations,
}

/// Validation rules for messages

/// Validate message creation
pub fn validate_create_message(
    message: Message,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // Verify sender matches agent creating entry
    if message.sender != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Sender must match creating agent".to_string(),
        ));
    }

    // Verify timestamp is reasonable (within 5 minutes of action timestamp)
    let action_time = action.timestamp.as_micros() as u64;
    let five_minutes = 300_000_000_u64; // 5 min in microseconds

    if message.sent_at > action_time + five_minutes
        || message.sent_at < action_time.saturating_sub(five_minutes)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Message timestamp out of valid range".to_string(),
        ));
    }

    // Verify encrypted content is not empty
    if message.encrypted_content.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Message content cannot be empty".to_string(),
        ));
    }

    // Verify encrypted content is not too large (10KB limit)
    if message.encrypted_content.len() > 10_000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Message content too large (max 10KB)".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate conversation creation
pub fn validate_create_conversation(
    conversation: Conversation,
) -> ExternResult<ValidateCallbackResult> {
    // Verify at least 2 participants
    if conversation.participants.len() < 2 {
        return Ok(ValidateCallbackResult::Invalid(
            "Conversation must have at least 2 participants".to_string(),
        ));
    }

    // Verify subject is not empty
    if conversation.subject.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Conversation subject cannot be empty".to_string(),
        ));
    }

    // Verify timestamps are valid
    if conversation.last_activity_at < conversation.started_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Last activity cannot be before start time".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate conversation update -- re-derives block_conversation's own real, previously
/// coordinator-only invariant ("must be a participant") at the DHT level, and closes a real
/// coordinator bug where archive_conversation had no check at all (any agent could archive
/// any conversation). Only status/last_message_hash/message_count/unread_counts/
/// last_activity_at may change; everything else must stay byte-identical to the original.
fn validate_update_conversation(
    conversation: Conversation,
    action: Update,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    if let ValidateCallbackResult::Invalid(reason) =
        validate_create_conversation(conversation.clone())?
    {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let Some(original): Option<Conversation> = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
    else {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid original Conversation entry".to_string(),
        ));
    };

    if conversation.participants != original.participants
        || conversation.listing_hash != original.listing_hash
        || conversation.transaction_hash != original.transaction_hash
        || conversation.subject != original.subject
        || conversation.first_message_hash != original.first_message_hash
        || conversation.started_at != original.started_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Conversation updates may only change status/last_message_hash/message_count/\
             unread_counts/last_activity_at -- all other fields must be unchanged"
                .to_string(),
        ));
    }

    if !original.participants.contains(&action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "Only a participant may update this conversation".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate read receipt creation
pub fn validate_create_read_receipt(
    receipt: ReadReceipt,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // Verify reader matches creating agent
    if receipt.reader != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Reader must match creating agent".to_string(),
        ));
    }

    // Verify timestamp is reasonable
    let action_time = action.timestamp.as_micros() as u64;
    let five_minutes = 300_000_000_u64;

    if receipt.read_at > action_time + five_minutes {
        return Ok(ValidateCallbackResult::Invalid(
            "Read timestamp cannot be in the future".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

// Validation function dispatcher
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Message(message) => validate_create_message(message, &action),
                EntryTypes::Conversation(conversation) => {
                    validate_create_conversation(conversation)
                }
                EntryTypes::ReadReceipt(receipt) => validate_create_read_receipt(receipt, &action),
            },
            // Message/ReadReceipt are confirmed create-only (no coordinator function ever
            // calls update_entry on either type) -- reject outright rather than leave the
            // previous unbound dead-code path (P0 wide-open RegisterUpdate gap, confirmed
            // dozens of times elsewhere in this pass). Conversation has a real update path
            // (archive_conversation/block_conversation/update_conversation_metadata/
            // update_conversation_unread_count) -- send_message now verifies the sender is
            // a real participant before touching a conversation, establishing the invariant
            // this bind re-derives at the DHT level (see
            // memory/mycelix_attribution_author_binding_jul8.md).
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::Message(_message) => Ok(ValidateCallbackResult::Invalid(
                    "Message entries cannot be updated".to_string(),
                )),
                EntryTypes::Conversation(conversation) => {
                    validate_update_conversation(conversation, action, original_action_hash)
                }
                EntryTypes::ReadReceipt(_receipt) => Ok(ValidateCallbackResult::Invalid(
                    "ReadReceipt entries cannot be updated".to_string(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(update_entry) => match update_entry {
            OpUpdate::Entry { app_entry, action } => match app_entry {
                EntryTypes::Message(_message) => Ok(ValidateCallbackResult::Invalid(
                    "Message entries cannot be updated".to_string(),
                )),
                EntryTypes::Conversation(conversation) => {
                    let original_action_hash = action.original_action_address.clone();
                    validate_update_conversation(conversation, action, original_action_hash)
                }
                EntryTypes::ReadReceipt(_receipt) => Ok(ValidateCallbackResult::Invalid(
                    "ReadReceipt entries cannot be updated".to_string(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(_delete_entry) => {
            // Allow deletion of messages, conversations, receipts
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => {
            // Validate link creation based on type
            match link_type {
                LinkTypes::AgentToConversations => Ok(ValidateCallbackResult::Valid),
                LinkTypes::AgentToSentMessages => Ok(ValidateCallbackResult::Valid),
                LinkTypes::AgentToReceivedMessages => Ok(ValidateCallbackResult::Valid),
                LinkTypes::ConversationToMessages => Ok(ValidateCallbackResult::Valid),
                LinkTypes::MessageToReadReceipts => Ok(ValidateCallbackResult::Valid),
                LinkTypes::ListingToConversations => Ok(ValidateCallbackResult::Valid),
                LinkTypes::TransactionToConversations => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Proves `validate_update_conversation`'s P0 author-binding fix: an update from an agent
/// who isn't a participant is rejected (previously archive_conversation had no
/// authorization check at all -- any agent could archive any conversation). Mocks the HDI
/// host's `must_get_valid_record` so this runs as a plain `cargo test`, no live conductor
/// needed.
#[cfg(test)]
mod tests {
    use super::*;

    struct MockRecordHdi {
        records: std::collections::HashMap<ActionHash, Record>,
    }

    impl hdi::hdi::HdiT for MockRecordHdi {
        fn must_get_valid_record(&self, input: MustGetValidRecordInput) -> ExternResult<Record> {
            self.records
                .get(&input.0)
                .cloned()
                .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("no such record in mock".into())))
        }
        fn verify_signature(&self, _: VerifySignature) -> ExternResult<bool> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_entry(&self, _: MustGetEntryInput) -> ExternResult<EntryHashed> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_action(&self, _: MustGetActionInput) -> ExternResult<SignedActionHashed> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_agent_activity(
            &self,
            _: MustGetAgentActivityInput,
        ) -> ExternResult<Vec<RegisterAgentActivity>> {
            unimplemented!("not exercised by this fix")
        }
        fn dna_info(&self, _: ()) -> ExternResult<DnaInfo> {
            unimplemented!("not exercised by this fix")
        }
        fn zome_info(&self, _: ()) -> ExternResult<ZomeInfo> {
            unimplemented!("not exercised by this fix")
        }
        fn trace(&self, _: TraceMsg) -> ExternResult<()> {
            unimplemented!("not exercised by this fix")
        }
        fn x_salsa20_poly1305_decrypt(
            &self,
            _: XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by this fix")
        }
        fn x_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: X25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by this fix")
        }
        fn ed_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: Ed25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<XSalsa20Poly1305Data> {
            unimplemented!("not exercised by this fix")
        }
    }

    fn wrap_entry_record<T>(author: AgentPubKey, value: T) -> Record
    where
        T: TryInto<SerializedBytes>,
        <T as TryInto<SerializedBytes>>::Error: std::fmt::Debug,
    {
        let entry = Entry::App(AppEntryBytes::try_from(value.try_into().unwrap()).unwrap());
        let action = Action::Create(Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![1; 36]),
            weight: Default::default(),
        });
        let hashed = HoloHashed::from_content_sync(action);
        let signed_action = SignedActionHashed::with_presigned(hashed, Signature([0; 64]));
        Record::new(signed_action, Some(entry))
    }

    fn test_conversation(participants: Vec<AgentPubKey>) -> Conversation {
        Conversation {
            participants,
            listing_hash: None,
            transaction_hash: None,
            subject: "Question about listing".to_string(),
            first_message_hash: ActionHash::from_raw_36(vec![10; 36]),
            last_message_hash: ActionHash::from_raw_36(vec![10; 36]),
            message_count: 1,
            unread_counts: vec![],
            started_at: 0,
            last_activity_at: 0,
            status: ConversationStatus::Active,
        }
    }

    fn test_update_action(author: AgentPubKey, original_hash: ActionHash) -> Update {
        Update {
            author,
            timestamp: Timestamp::from_micros(100),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![20; 36]),
            original_action_address: original_hash,
            original_entry_address: EntryHash::from_raw_36(vec![22; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![23; 36]),
            weight: Default::default(),
        }
    }

    #[test]
    fn conversation_update_by_non_participant_is_rejected() {
        let p1 = AgentPubKey::from_raw_36(vec![1; 36]);
        let p2 = AgentPubKey::from_raw_36(vec![2; 36]);
        let original = test_conversation(vec![p1.clone(), p2]);
        let original_hash = ActionHash::from_raw_36(vec![30; 36]);
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                original_hash.clone(),
                wrap_entry_record(p1, original.clone()),
            )]),
        });

        let mut updated = original.clone();
        updated.status = ConversationStatus::Archived;

        let outsider = AgentPubKey::from_raw_36(vec![99; 36]);
        let action = test_update_action(outsider, original_hash.clone());
        let result = validate_update_conversation(updated, action, original_hash).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(_)),
            "only a participant may update a conversation -- previously archive_conversation \
             had no authorization check at all"
        );
    }

    #[test]
    fn conversation_update_by_a_participant_is_accepted() {
        let p1 = AgentPubKey::from_raw_36(vec![1; 36]);
        let p2 = AgentPubKey::from_raw_36(vec![2; 36]);
        let original = test_conversation(vec![p1.clone(), p2.clone()]);
        let original_hash = ActionHash::from_raw_36(vec![30; 36]);
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                original_hash.clone(),
                wrap_entry_record(p1, original.clone()),
            )]),
        });

        let mut updated = original.clone();
        updated.status = ConversationStatus::Archived;

        let action = test_update_action(p2, original_hash.clone());
        let result = validate_update_conversation(updated, action, original_hash).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }
}
