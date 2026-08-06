// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
#![deny(unsafe_code)]
//! Mail Messages Integrity Zome
//!
//! Defines entry types and validation rules for decentralized email on Holochain DHT.
//! All emails are encrypted, stored as DHT entries, and delivered via P2P signals.

use hdi::prelude::*;
use keys_types::{HybridKeyBundleV2, HybridKeyStateV2, hybrid_key_id};
use mail_leptos_types::protocol::{
    AuthenticatedMetadataV1, EncryptedEnvelopeV2HybridPqc, EncryptionKeyId,
    ML_DSA_65_SIGNATURE_BYTES, MessageId,
};

const MAX_CHUNK_SIZE: usize = 10 * 1024 * 1024;
const MAX_TOTAL_CHUNKS: u32 = 1000;
const SHA256_LEN: usize = 32;
const ED25519_SIG_LEN: usize = 64;
const DILITHIUM3_SIG_LEN: usize = 3293;
const DILITHIUM2_SIG_LEN: usize = 2420;
const X25519_KEY_LEN: usize = 32;
const KYBER1024_KEY_LEN: usize = 1568;
const KYBER768_KEY_LEN: usize = 1088;
const MAX_ENCRYPTED_SUBJECT_BYTES: usize = 64 * 1024;
const MAX_ENCRYPTED_BODY_BYTES: usize = 2 * 1024 * 1024;
const MAX_MESSAGE_ID_BYTES: usize = 512;

// Phase 0.8 client-authoritative timestamp bounds.
// `email.timestamp` is client-signed (RFC 5322 Date:). `action.timestamp` is
// hc-chain-assigned when the entry is committed. We bound the skew between
// them to prevent spoofing without rejecting legitimate offline mail.
const MAX_FUTURE_SKEW_MICROS: i64 = 5 * 60 * 1_000_000; // 5 min — clock drift tolerance
const MAX_PAST_SKEW_MICROS: i64 = 30 * 86_400 * 1_000_000; // 30 days — offline compose tolerance

pub fn email_signing_content(email: &EncryptedEmail) -> Vec<u8> {
    let mut content = Vec::with_capacity(
        512 + email.encrypted_subject.len()
            + email.encrypted_body.len()
            + email.encrypted_attachments.len(),
    );
    content.extend_from_slice(b"mycelix-pulse/encrypted-email/v1\0");
    content.extend_from_slice(email.sender.get_raw_39());
    content.extend_from_slice(email.recipient.get_raw_39());
    append_signing_bytes(&mut content, &email.encrypted_subject);
    append_signing_bytes(&mut content, &email.encrypted_body);
    append_signing_bytes(&mut content, &email.encrypted_attachments);
    append_signing_bytes(&mut content, &email.ephemeral_pubkey);
    content.extend_from_slice(&email.nonce);
    append_signing_bytes(&mut content, email.crypto_suite.key_exchange.as_bytes());
    append_signing_bytes(&mut content, email.crypto_suite.symmetric.as_bytes());
    append_signing_bytes(&mut content, email.crypto_suite.signature.as_bytes());
    append_signing_bytes(&mut content, email.message_id.as_bytes());
    append_optional_signing_string(&mut content, email.in_reply_to.as_deref());
    content.extend_from_slice(&(email.references.len() as u32).to_be_bytes());
    for reference in &email.references {
        append_signing_bytes(&mut content, reference.as_bytes());
    }
    content.extend_from_slice(&email.timestamp.as_micros().to_be_bytes());
    content.push(match &email.priority {
        EmailPriority::Low => 1,
        EmailPriority::Normal => 2,
        EmailPriority::High => 3,
        EmailPriority::Urgent => 4,
    });
    content.push(u8::from(email.read_receipt_requested));
    match email.expires_at {
        Some(timestamp) => {
            content.push(1);
            content.extend_from_slice(&timestamp.as_micros().to_be_bytes());
        }
        None => content.push(0),
    }
    content
}

fn append_signing_bytes(content: &mut Vec<u8>, bytes: &[u8]) {
    content.extend_from_slice(&(bytes.len() as u32).to_be_bytes());
    content.extend_from_slice(bytes);
}

fn append_optional_signing_string(content: &mut Vec<u8>, value: Option<&str>) {
    match value {
        Some(value) => {
            content.push(1);
            append_signing_bytes(content, value.as_bytes());
        }
        None => content.push(0),
    }
}

pub fn receipt_signing_content(
    email_hash: &ActionHash,
    reader: &AgentPubKey,
    read_at: &Timestamp,
) -> Vec<u8> {
    let mut content = Vec::with_capacity(128);
    content.push(0x01);
    content.extend_from_slice(email_hash.get_raw_39());
    content.extend_from_slice(reader.get_raw_39());
    content.extend_from_slice(&read_at.as_micros().to_le_bytes());
    content
}

pub fn delivery_receipt_signing_content(
    email_hash: &ActionHash,
    recipient: &AgentPubKey,
    delivered_at: &Timestamp,
) -> Vec<u8> {
    let mut content = Vec::with_capacity(128);
    content.push(0x01);
    content.extend_from_slice(email_hash.get_raw_39());
    content.extend_from_slice(recipient.get_raw_39());
    content.extend_from_slice(&delivered_at.as_micros().to_le_bytes());
    content
}

/// Email message stored on DHT
/// Content is always encrypted - only metadata is visible for routing
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EncryptedEmail {
    /// Sender's agent public key
    pub sender: AgentPubKey,
    /// Recipient's agent public key
    pub recipient: AgentPubKey,
    /// Encrypted subject (ChaCha20-Poly1305)
    pub encrypted_subject: Vec<u8>,
    /// Encrypted body content
    pub encrypted_body: Vec<u8>,
    /// Encrypted attachments manifest (references to attachment entries)
    pub encrypted_attachments: Vec<u8>,
    /// Ephemeral public key for decryption (X25519 or Kyber)
    pub ephemeral_pubkey: Vec<u8>,
    /// Nonce used for encryption
    pub nonce: [u8; 24],
    /// Digital signature of content hash (Dilithium or Ed25519)
    pub signature: Vec<u8>,
    /// Algorithm identifiers
    pub crypto_suite: CryptoSuite,
    /// Message ID for threading
    pub message_id: String,
    /// In-reply-to message ID (for threads)
    pub in_reply_to: Option<String>,
    /// References (thread chain)
    pub references: Vec<String>,
    /// Timestamp (author's claimed time)
    pub timestamp: Timestamp,
    /// Priority level
    pub priority: EmailPriority,
    /// Read receipt requested
    pub read_receipt_requested: bool,
    /// Expiration time (for ephemeral messages)
    pub expires_at: Option<Timestamp>,
}

/// The immutable V2 envelope. It is a separate entry so V1 remains readable
/// and schema interpretation never depends on algorithm-name heuristics.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EncryptedEmailV2 {
    pub version: u16,
    pub cipher_suite: String,
    pub message_id: [u8; 32],
    pub sender: AgentPubKey,
    pub recipient: AgentPubKey,
    pub sender_mldsa_key_id: [u8; 32],
    pub recipient_hybrid_key_id: [u8; 32],
    /// ActionHash of the sender's published `HybridKeyBundleV2` entry (a DHT
    /// zome_adapter lookup pointer, not itself authenticated — see the field
    /// doc on `mail_leptos_types::protocol::EncryptedEnvelopeV2HybridPqc::sender_mldsa_bundle_hash`).
    pub sender_mldsa_bundle_hash: ActionHash,
    /// ActionHash of the recipient's published `HybridKeyBundleV2` entry, at
    /// send time — mirrors `sender_mldsa_bundle_hash`'s lookup-pointer-only
    /// contract (see `verify_recipient_key_state`).
    pub recipient_bundle_hash: ActionHash,
    pub x25519_ephemeral_public_key: [u8; 32],
    pub ml_kem_ciphertext: Vec<u8>,
    pub nonce: [u8; 12],
    pub ciphertext: Vec<u8>,
    pub in_reply_to: Option<[u8; 32]>,
    pub thread_id: Option<[u8; 32]>,
    pub created_at_micros: i64,
    pub agent_signature: Vec<u8>,
    pub ml_dsa_signature: Vec<u8>,
}

impl EncryptedEmailV2 {
    pub fn protocol_envelope(&self) -> EncryptedEnvelopeV2HybridPqc {
        EncryptedEnvelopeV2HybridPqc {
            version: self.version,
            cipher_suite: self.cipher_suite.clone(),
            message_id: MessageId(self.message_id),
            sender_agent: self.sender.get_raw_39().to_vec(),
            recipient_agent: self.recipient.get_raw_39().to_vec(),
            sender_mldsa_key_id: EncryptionKeyId(self.sender_mldsa_key_id),
            sender_mldsa_bundle_hash: self.sender_mldsa_bundle_hash.get_raw_39().to_vec(),
            recipient_bundle_hash: self.recipient_bundle_hash.get_raw_39().to_vec(),
            recipient_hybrid_key_id: EncryptionKeyId(self.recipient_hybrid_key_id),
            x25519_ephemeral_public_key: self.x25519_ephemeral_public_key,
            ml_kem_ciphertext: self.ml_kem_ciphertext.clone(),
            nonce: self.nonce,
            ciphertext: self.ciphertext.clone(),
            metadata: AuthenticatedMetadataV1 {
                in_reply_to: self.in_reply_to.map(MessageId),
                thread_id: self.thread_id,
            },
            created_at_micros: self.created_at_micros,
            agent_signature: self.agent_signature.clone(),
            ml_dsa_signature: self.ml_dsa_signature.clone(),
        }
    }
}

/// Cryptographic algorithms used
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Default)]
pub struct CryptoSuite {
    /// Key exchange algorithm (x25519, kyber1024)
    pub key_exchange: String,
    /// Symmetric encryption (chacha20-poly1305, aes-256-gcm)
    pub symmetric: String,
    /// Signature algorithm (ed25519, dilithium3)
    pub signature: String,
}

/// Email priority levels
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Default)]
pub enum EmailPriority {
    Low,
    #[default]
    Normal,
    High,
    Urgent,
}

/// Large attachment stored separately on DHT
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EncryptedAttachment {
    /// Reference to parent email
    pub email_hash: ActionHash,
    /// Encrypted filename
    pub encrypted_filename: Vec<u8>,
    /// Encrypted MIME type
    pub encrypted_mime_type: Vec<u8>,
    /// Encrypted content (chunked for large files)
    pub encrypted_content: Vec<u8>,
    /// Chunk index (for multi-part attachments)
    pub chunk_index: u32,
    /// Total chunks
    pub total_chunks: u32,
    /// Content hash for integrity verification
    pub content_hash: Vec<u8>,
    /// Encryption nonce
    pub nonce: [u8; 24],
}

/// Email folder/label assignment
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmailFolder {
    /// Owner agent
    pub owner: AgentPubKey,
    /// Folder name (encrypted)
    pub encrypted_name: Vec<u8>,
    /// Folder color/icon metadata
    pub metadata: Option<Vec<u8>>,
    /// Is system folder (inbox, sent, drafts, trash)
    pub is_system: bool,
    /// Sort order
    pub sort_order: i32,
}

/// Email state for a specific agent (read, starred, etc.)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmailState {
    /// Email entry hash
    pub email_hash: ActionHash,
    /// Owner of this state
    pub owner: AgentPubKey,
    /// Read status
    pub is_read: bool,
    /// Starred/flagged
    pub is_starred: bool,
    /// Folder assignments
    pub folders: Vec<ActionHash>,
    /// Labels (encrypted)
    pub encrypted_labels: Vec<Vec<u8>>,
    /// Snoozed until
    pub snoozed_until: Option<Timestamp>,
    /// Archived
    pub is_archived: bool,
    /// Trashed
    pub is_trashed: bool,
    /// Trash timestamp (for auto-delete)
    pub trashed_at: Option<Timestamp>,
}

/// Draft email (not yet sent)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmailDraft {
    /// Owner/author
    pub owner: AgentPubKey,
    /// Intended recipients
    pub recipients: Vec<AgentPubKey>,
    /// CC recipients
    pub cc: Vec<AgentPubKey>,
    /// BCC recipients (stored separately, not shared)
    pub bcc: Vec<AgentPubKey>,
    /// Encrypted subject
    pub encrypted_subject: Vec<u8>,
    /// Encrypted body
    pub encrypted_body: Vec<u8>,
    /// Attachment references
    pub attachments: Vec<ActionHash>,
    /// Reply-to email
    pub in_reply_to: Option<ActionHash>,
    /// Last modified
    pub updated_at: Timestamp,
    /// Scheduled send time
    pub scheduled_for: Option<Timestamp>,
}

/// Read receipt confirmation
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ReadReceipt {
    /// Original email hash
    pub email_hash: ActionHash,
    /// Reader agent
    pub reader: AgentPubKey,
    /// When read
    pub read_at: Timestamp,
    /// Signature proving authenticity
    pub signature: Vec<u8>,
}

/// Delivery receipt (email received by recipient's node)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DeliveryReceipt {
    /// Original email hash
    pub email_hash: ActionHash,
    /// Recipient who received it
    pub recipient: AgentPubKey,
    /// Delivery timestamp
    pub delivered_at: Timestamp,
    /// Signature
    pub signature: Vec<u8>,
}

/// Email thread grouping
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmailThread {
    /// Thread ID (from first message)
    pub thread_id: String,
    /// Subject (encrypted, from first message)
    pub encrypted_subject: Vec<u8>,
    /// Participants
    pub participants: Vec<AgentPubKey>,
    /// Message count
    pub message_count: u32,
    /// Last activity
    pub last_activity: Timestamp,
}

/// Link types for email graph
#[hdk_link_types]
pub enum LinkTypes {
    /// Agent -> their received emails
    AgentToInbox,
    /// Agent -> their sent emails
    AgentToSent,
    /// Agent -> their drafts
    AgentToDrafts,
    /// Agent -> their folders
    AgentToFolders,
    /// Folder -> emails in folder
    FolderToEmails,
    /// Email -> its attachments
    EmailToAttachments,
    /// Email -> read receipts
    EmailToReadReceipts,
    /// Email -> delivery receipts
    EmailToDeliveryReceipts,
    /// Email -> its state (per agent)
    EmailToState,
    /// Thread -> emails in thread
    ThreadToEmails,
    /// Agent -> threads they're part of
    AgentToThreads,
    /// Email -> reply emails
    EmailToReplies,
    /// Scheduled emails pending send
    AgentToScheduled,
    /// Anchor for global discovery (optional, for public emails)
    AnchorToPublicEmails,
    /// Agent -> V2 messages they sent. Kept separate from the V1 namespace.
    AgentToSentV2,
    /// Agent -> V2 messages they received. Kept separate from the V1 namespace.
    AgentToInboxV2,
}

/// Entry type definitions
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(required_validations = 3)]
    EncryptedEmail(EncryptedEmail),
    #[entry_type(required_validations = 3)]
    EncryptedEmailV2(EncryptedEmailV2),
    #[entry_type(required_validations = 2)]
    EncryptedAttachment(EncryptedAttachment),
    #[entry_type(required_validations = 2)]
    EmailFolder(EmailFolder),
    #[entry_type(required_validations = 2)]
    EmailState(EmailState),
    #[entry_type(required_validations = 1)]
    EmailDraft(EmailDraft),
    #[entry_type(required_validations = 2)]
    ReadReceipt(ReadReceipt),
    #[entry_type(required_validations = 2)]
    DeliveryReceipt(DeliveryReceipt),
    #[entry_type(required_validations = 2)]
    EmailThread(EmailThread),
}

/// Validation callbacks
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => validate_create_entry(app_entry, action),
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => validate_update_entry(app_entry, action),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            tag,
            action,
        } => validate_create_link(link_type, base_address, target_address, tag, action),
        FlatOp::RegisterDeleteLink {
            link_type,
            original_action,
            base_address,
            target_address,
            tag,
            action,
        } => validate_delete_link(
            link_type,
            original_action,
            base_address,
            target_address,
            tag,
            action,
        ),
        FlatOp::StoreRecord(store_record) => match store_record {
            OpRecord::CreateEntry { app_entry, action } => validate_create_entry(app_entry, action),
            OpRecord::UpdateEntry {
                app_entry, action, ..
            } => validate_update_entry(app_entry, action),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_entry(
    entry: EntryTypes,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::EncryptedEmail(email) => validate_encrypted_email(&email, &action),
        EntryTypes::EncryptedEmailV2(email) => validate_encrypted_email_v2(&email, &action),
        EntryTypes::EncryptedAttachment(attachment) => validate_attachment(&attachment, &action),
        EntryTypes::EmailFolder(folder) => validate_folder(&folder, &action),
        EntryTypes::EmailState(state) => validate_email_state(&state, &action),
        EntryTypes::EmailDraft(draft) => validate_draft(&draft, &action),
        EntryTypes::ReadReceipt(receipt) => validate_read_receipt(&receipt, &action),
        EntryTypes::DeliveryReceipt(receipt) => validate_delivery_receipt(&receipt, &action),
        EntryTypes::EmailThread(thread) => validate_thread(&thread, &action),
    }
}

fn validate_update_entry(
    entry: EntryTypes,
    action: Update,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        // Emails are immutable once sent
        EntryTypes::EncryptedEmail(_) => Ok(ValidateCallbackResult::Invalid(
            "Sent emails cannot be modified".to_string(),
        )),
        EntryTypes::EncryptedEmailV2(_) => Ok(ValidateCallbackResult::Invalid(
            "Sent V2 emails cannot be modified".to_string(),
        )),
        // Drafts can be updated by owner
        EntryTypes::EmailDraft(draft) => {
            if draft.owner != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only draft owner can update".to_string(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        // State can be updated by owner
        EntryTypes::EmailState(state) => {
            if state.owner != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only state owner can update".to_string(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        // Folders can be updated by owner
        EntryTypes::EmailFolder(folder) => {
            if folder.owner != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only folder owner can update".to_string(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        // Receipts are immutable
        EntryTypes::ReadReceipt(_) | EntryTypes::DeliveryReceipt(_) => Ok(
            ValidateCallbackResult::Invalid("Receipts cannot be modified".to_string()),
        ),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Host-independent structural checks for a V2 entry: ciphertext/signature
/// component lengths and the canonical transcript encoding. These never call
/// an HDI host function, so they run in a plain `cargo test` without a live
/// conductor. Returns the transcript to sign/verify on success.
///
/// Deliberately NOT checked here: the ML-DSA-65 signature's cryptographic
/// validity, and the sender/recipient key bundles' lifecycle state — both
/// need `must_get_valid_record`, unavailable without a live HDI host. See
/// `verify_ml_dsa_v2`/`verify_recipient_key_state` (called from
/// `validate_encrypted_email_v2`), which do enforce all of that on-chain.
fn validate_email_v2_structure(email: &EncryptedEmailV2) -> Result<Vec<u8>, String> {
    if email.ciphertext.len() < 16 || email.ciphertext.len() > MAX_ENCRYPTED_BODY_BYTES {
        return Err("Invalid V2 ciphertext length".into());
    }
    if email.ml_dsa_signature.len() != ML_DSA_65_SIGNATURE_BYTES {
        return Err("ML-DSA-65 signature must be 3309 bytes".into());
    }
    if email.agent_signature.len() != ED25519_SIG_LEN {
        return Err("Agent signature must be 64 bytes".into());
    }
    email
        .protocol_envelope()
        .canonical_signing_bytes()
        .map_err(|error| format!("Invalid V2 envelope: {error:?}"))
}

fn validate_encrypted_email_v2(
    email: &EncryptedEmailV2,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if email.sender != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "V2 sender must match action author".into(),
        ));
    }
    let transcript = match validate_email_v2_structure(email) {
        Ok(transcript) => transcript,
        Err(reason) => return Ok(ValidateCallbackResult::Invalid(reason)),
    };
    let mut signature = [0; 64];
    signature.copy_from_slice(&email.agent_signature);
    if !verify_signature_raw(
        email.sender.clone(),
        Signature(signature),
        transcript.clone(),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "V2 agent signature verification failed".into(),
        ));
    }
    if let Err(reason) = verify_ml_dsa_v2(email, &transcript)? {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }
    if let Err(reason) = verify_recipient_key_state(email)? {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }
    let created = email.created_at_micros;
    let action_time = action.timestamp.as_micros();
    if created.saturating_sub(action_time) > MAX_FUTURE_SKEW_MICROS
        || action_time.saturating_sub(created) > MAX_PAST_SKEW_MICROS
    {
        return Ok(ValidateCallbackResult::Invalid(
            "V2 created_at is outside the accepted action-time window".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Verify the ML-DSA-65 signature against the sender's historical key
/// bundle, fetched deterministically via `must_get_valid_record`. Closes the
/// gap documented in ADR-002/PULSE_V2_CRYPTO_SPEC.md: previously only the
/// signature's *length* was enforced on-chain, never its cryptographic
/// validity or the referenced bundle's lifecycle state — both were left to
/// the recipient client. `sender_mldsa_bundle_hash` is a lookup pointer
/// only, not itself trusted: every check below re-derives something the
/// sender already committed to in a way that can't be forged by pointing at
/// an arbitrary hash (see the field doc for the full rationale).
///
/// Returns `Ok(Err(reason))` for a genuine rejection (surfaced as
/// `ValidateCallbackResult::Invalid`), and lets `must_get_valid_record`'s own
/// `Err` propagate via `?` so an unresolved dependency triggers Holochain's
/// retry-validation-later behavior instead of a hard rejection.
/// Fetch a `HybridKeyBundleV2` via its lookup-pointer `ActionHash` and
/// independently verify it before trusting it: that it was really authored
/// by `expected_author` (a forged/wrong pointer can only ever point at
/// someone else's own genuine record, never fabricate authorship), and that
/// its own content hash matches `expected_key_id` (something the caller
/// already signed/committed to, so the pointer itself is never trusted for
/// *which* key it names — only for finding it). Shared by
/// `verify_ml_dsa_v2` and `verify_recipient_key_state`, which each add their
/// own check on top (signature validity, lifecycle state respectively).
fn fetch_and_verify_bundle(
    bundle_hash: ActionHash,
    expected_author: &AgentPubKey,
    expected_key_id: [u8; 32],
    pointer_field_name: &str,
) -> ExternResult<Result<HybridKeyBundleV2, String>> {
    let record = must_get_valid_record(bundle_hash)?;
    if record.action().author() != expected_author {
        return Ok(Err(format!(
            "{pointer_field_name} was not authored by the expected agent"
        )));
    }
    let Some(entry) = record.entry().as_option() else {
        return Ok(Err(format!("{pointer_field_name} record has no entry")));
    };
    let Entry::App(app_bytes) = entry else {
        return Ok(Err(format!("{pointer_field_name} is not an app entry")));
    };
    let bundle = match HybridKeyBundleV2::try_from(SerializedBytes::from(app_bytes.clone())) {
        Ok(bundle) => bundle,
        Err(error) => {
            return Ok(Err(format!(
                "{pointer_field_name} failed to deserialize as a hybrid key bundle: {error}"
            )));
        }
    };
    if hybrid_key_id(&bundle) != expected_key_id {
        return Ok(Err(format!(
            "{pointer_field_name} does not resolve to the expected key id"
        )));
    }
    Ok(Ok(bundle))
}

fn verify_ml_dsa_v2(
    email: &EncryptedEmailV2,
    transcript: &[u8],
) -> ExternResult<Result<(), String>> {
    let bundle = match fetch_and_verify_bundle(
        email.sender_mldsa_bundle_hash.clone(),
        &email.sender,
        email.sender_mldsa_key_id,
        "sender_mldsa_bundle_hash",
    )? {
        Ok(bundle) => bundle,
        Err(reason) => return Ok(Err(reason)),
    };
    if bundle.state != HybridKeyStateV2::Active {
        return Ok(Err(
            "sender's ML-DSA key bundle is not in the Active state".into()
        ));
    }

    use ml_dsa::{MlDsa65, Signature as MlDsaSignature, VerifyingKey, common::KeyInit};
    use signature::Verifier;

    let Ok(verifying_key) = VerifyingKey::<MlDsa65>::new_from_slice(&bundle.ml_dsa_65_public_key)
    else {
        return Ok(Err(
            "sender's ML-DSA public key has an invalid length".into()
        ));
    };
    let Ok(ml_dsa_signature) =
        MlDsaSignature::<MlDsa65>::try_from(email.ml_dsa_signature.as_slice())
    else {
        return Ok(Err("ML-DSA-65 signature has an invalid length".into()));
    };
    if verifying_key.verify(transcript, &ml_dsa_signature).is_err() {
        return Ok(Err("ML-DSA-65 signature verification failed".into()));
    }
    Ok(Ok(()))
}

/// Closes the recipient-side half of the ML-DSA gap disclosed in
/// ADR-002/PULSE_V2_CRYPTO_SPEC.md: the sender-side fix (`verify_ml_dsa_v2`)
/// checked the *sender's* key state but left the *recipient's* unenforced —
/// a message to a recipient whose key had since been revoked/expired was
/// still accepted onto the DHT, relying entirely on the recipient client
/// (`load_v2_inbox`) to catch it. `recipient_bundle_hash` is a lookup
/// pointer only, verified the same way as `sender_mldsa_bundle_hash` (see
/// `fetch_and_verify_bundle`).
fn verify_recipient_key_state(email: &EncryptedEmailV2) -> ExternResult<Result<(), String>> {
    let bundle = match fetch_and_verify_bundle(
        email.recipient_bundle_hash.clone(),
        &email.recipient,
        email.recipient_hybrid_key_id,
        "recipient_bundle_hash",
    )? {
        Ok(bundle) => bundle,
        Err(reason) => return Ok(Err(reason)),
    };
    if bundle.state != HybridKeyStateV2::Active {
        return Ok(Err(
            "recipient's hybrid key bundle is not in the Active state".into(),
        ));
    }
    Ok(Ok(()))
}

fn validate_encrypted_email(
    email: &EncryptedEmail,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // Sender must be the author
    if email.sender != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Email sender must match action author".to_string(),
        ));
    }

    // Must have encrypted content
    if email.encrypted_body.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Email body cannot be empty".to_string(),
        ));
    }
    if email.encrypted_subject.len() > MAX_ENCRYPTED_SUBJECT_BYTES {
        return Ok(ValidateCallbackResult::Invalid(
            "Encrypted subject exceeds 64 KiB".to_string(),
        ));
    }
    if email.encrypted_body.len() > MAX_ENCRYPTED_BODY_BYTES {
        return Ok(ValidateCallbackResult::Invalid(
            "Encrypted body exceeds 2 MiB".to_string(),
        ));
    }
    if email.message_id.is_empty() || email.message_id.len() > MAX_MESSAGE_ID_BYTES {
        return Ok(ValidateCallbackResult::Invalid(
            "Message ID must be between 1 and 512 bytes".to_string(),
        ));
    }

    // Must have valid nonce
    if email.nonce == [0u8; 24] {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid encryption nonce".to_string(),
        ));
    }

    // Must have signature
    if email.signature.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Email must be signed".to_string(),
        ));
    }

    // Check expiration if set
    if let Some(expires) = email.expires_at {
        if expires <= email.timestamp {
            return Ok(ValidateCallbackResult::Invalid(
                "Expiration must be after timestamp".to_string(),
            ));
        }
    }

    // Phase 0.8 — bound client-set timestamp against chain-assigned action
    // timestamp. Future-dated emails (> 5 min ahead) are a spam vector (always
    // top of inbox). Unreasonably-old emails (> 30 d behind) are a replay
    // vector. Legitimate offline mail composed hours/days ago still validates.
    let email_ts = email.timestamp.as_micros();
    let action_ts = action.timestamp.as_micros();
    let future_skew = email_ts.saturating_sub(action_ts);
    if future_skew > MAX_FUTURE_SKEW_MICROS {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Email timestamp is {} us in the future (max {})",
            future_skew, MAX_FUTURE_SKEW_MICROS
        )));
    }
    let past_skew = action_ts.saturating_sub(email_ts);
    if past_skew > MAX_PAST_SKEW_MICROS {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Email timestamp is {} us in the past (max {})",
            past_skew, MAX_PAST_SKEW_MICROS
        )));
    }

    // Validate crypto suite
    let valid_key_exchange = ["x25519", "kyber1024", "kyber768"];
    let valid_symmetric = ["chacha20-poly1305", "aes-256-gcm"];
    let valid_signature = ["ed25519", "dilithium3", "dilithium2"];

    if !valid_key_exchange.contains(&email.crypto_suite.key_exchange.as_str()) {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid key exchange algorithm".to_string(),
        ));
    }
    if !valid_symmetric.contains(&email.crypto_suite.symmetric.as_str()) {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid symmetric encryption algorithm".to_string(),
        ));
    }
    if !valid_signature.contains(&email.crypto_suite.signature.as_str()) {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid signature algorithm".to_string(),
        ));
    }

    // Validate ephemeral key length matches key exchange algorithm
    let expected_key_len = match email.crypto_suite.key_exchange.as_str() {
        "x25519" => X25519_KEY_LEN,
        "kyber1024" => KYBER1024_KEY_LEN,
        "kyber768" => KYBER768_KEY_LEN,
        _ => {
            return Ok(ValidateCallbackResult::Invalid(
                "Unknown key exchange for ephemeral key validation".to_string(),
            ));
        }
    };
    if email.ephemeral_pubkey.len() != expected_key_len {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Ephemeral pubkey length {} does not match expected {} for {}",
            email.ephemeral_pubkey.len(),
            expected_key_len,
            email.crypto_suite.key_exchange
        )));
    }

    // Validate signature length and verify for Ed25519
    match email.crypto_suite.signature.as_str() {
        "ed25519" => {
            if email.signature.len() != ED25519_SIG_LEN {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Ed25519 signature must be {} bytes",
                    ED25519_SIG_LEN
                )));
            }
            // Verify signature against signing content
            let signing_content = email_signing_content(email);
            let mut sig_bytes = [0u8; 64];
            sig_bytes.copy_from_slice(&email.signature);
            let sig = Signature(sig_bytes);
            if !verify_signature_raw(email.sender.clone(), sig, signing_content)? {
                return Ok(ValidateCallbackResult::Invalid(
                    "Email signature verification failed".to_string(),
                ));
            }
        }
        "dilithium3" => {
            if email.signature.len() != DILITHIUM3_SIG_LEN {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Dilithium3 signature must be {} bytes",
                    DILITHIUM3_SIG_LEN
                )));
            }
            // Dilithium verification requires PQC library, validated at application layer
        }
        "dilithium2" => {
            if email.signature.len() != DILITHIUM2_SIG_LEN {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Dilithium2 signature must be {} bytes",
                    DILITHIUM2_SIG_LEN
                )));
            }
        }
        _ => {}
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_attachment(
    attachment: &EncryptedAttachment,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // The attachment's email_hash must resolve to a real email whose sender is
    // this attachment's own committer -- otherwise any agent could attach
    // arbitrary content to any email in the system (add_attachment takes the
    // whole EncryptedAttachment, including email_hash, as raw caller input).
    let email_record = must_get_valid_record(attachment.email_hash.clone())?;
    let email_entry = email_record.entry().as_option().ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "EncryptedAttachment.email_hash record has no entry".into()
        ))
    })?;
    let Entry::App(app_bytes) = email_entry else {
        return Ok(ValidateCallbackResult::Invalid(
            "EncryptedAttachment.email_hash target is not an app entry".into(),
        ));
    };
    let sender = if let Ok(email) =
        EncryptedEmail::try_from(SerializedBytes::from(app_bytes.clone()))
    {
        email.sender
    } else if let Ok(email) = EncryptedEmailV2::try_from(SerializedBytes::from(app_bytes.clone())) {
        email.sender
    } else {
        return Ok(ValidateCallbackResult::Invalid(
            "EncryptedAttachment.email_hash does not reference an EncryptedEmail or EncryptedEmailV2".into(),
        ));
    };
    if sender != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Attachment's email_hash is not owned by the attachment's committer".to_string(),
        ));
    }

    // Must have content
    if attachment.encrypted_content.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Attachment content cannot be empty".to_string(),
        ));
    }

    // Chunk size limit
    if attachment.encrypted_content.len() > MAX_CHUNK_SIZE {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Attachment chunk exceeds maximum size of {} bytes",
            MAX_CHUNK_SIZE
        )));
    }

    // Total chunks limit
    if attachment.total_chunks > MAX_TOTAL_CHUNKS {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Total chunks {} exceeds maximum of {}",
            attachment.total_chunks, MAX_TOTAL_CHUNKS
        )));
    }

    // Chunk index must be valid
    if attachment.chunk_index >= attachment.total_chunks {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid chunk index".to_string(),
        ));
    }

    // Must have SHA-256 content hash
    if attachment.content_hash.len() != SHA256_LEN {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Content hash must be {} bytes (SHA-256)",
            SHA256_LEN
        )));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_folder(folder: &EmailFolder, action: &Create) -> ExternResult<ValidateCallbackResult> {
    // Owner must be author
    if folder.owner != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Folder owner must match author".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_email_state(
    state: &EmailState,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // Owner must be author
    if state.owner != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "State owner must match author".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_draft(draft: &EmailDraft, action: &Create) -> ExternResult<ValidateCallbackResult> {
    // Owner must be author
    if draft.owner != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Draft owner must match author".to_string(),
        ));
    }

    // Must have at least one recipient
    if draft.recipients.is_empty() && draft.cc.is_empty() && draft.bcc.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Draft must have at least one recipient".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_read_receipt(
    receipt: &ReadReceipt,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // Reader must be author
    if receipt.reader != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Read receipt reader must match author".to_string(),
        ));
    }

    // Must have Ed25519 signature of correct length
    if receipt.signature.len() != ED25519_SIG_LEN {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Read receipt signature must be {} bytes (Ed25519)",
            ED25519_SIG_LEN
        )));
    }

    // Verify signature
    let signing_content =
        receipt_signing_content(&receipt.email_hash, &receipt.reader, &receipt.read_at);
    let mut sig_bytes = [0u8; 64];
    sig_bytes.copy_from_slice(&receipt.signature);
    let sig = Signature(sig_bytes);
    if !verify_signature_raw(receipt.reader.clone(), sig, signing_content)? {
        return Ok(ValidateCallbackResult::Invalid(
            "Read receipt signature verification failed".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_delivery_receipt(
    receipt: &DeliveryReceipt,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // Recipient must be author
    if receipt.recipient != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Delivery receipt recipient must match author".to_string(),
        ));
    }

    // Must have Ed25519 signature of correct length
    if receipt.signature.len() != ED25519_SIG_LEN {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Delivery receipt signature must be {} bytes (Ed25519)",
            ED25519_SIG_LEN
        )));
    }

    // Verify signature
    let signing_content = delivery_receipt_signing_content(
        &receipt.email_hash,
        &receipt.recipient,
        &receipt.delivered_at,
    );
    let mut sig_bytes = [0u8; 64];
    sig_bytes.copy_from_slice(&receipt.signature);
    let sig = Signature(sig_bytes);
    if !verify_signature_raw(receipt.recipient.clone(), sig, signing_content)? {
        return Ok(ValidateCallbackResult::Invalid(
            "Delivery receipt signature verification failed".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_thread(
    _thread: &EmailThread,
    _action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    // Threads are created automatically, minimal validation
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_link(
    link_type: LinkTypes,
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    _tag: LinkTag,
    action: CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    match link_type {
        LinkTypes::AgentToSent
        | LinkTypes::AgentToSentV2
        | LinkTypes::AgentToDrafts
        | LinkTypes::AgentToFolders
        | LinkTypes::AgentToThreads
        | LinkTypes::AgentToScheduled => {
            // Agent links must be created by the agent themselves
            let author_hash: AnyLinkableHash = action.author.into();
            if base_address != author_hash {
                return Ok(ValidateCallbackResult::Invalid(
                    "Agent link base must match action author".to_string(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        LinkTypes::AgentToInbox => validate_inbox_link(base_address, target_address, action),
        LinkTypes::AgentToInboxV2 => validate_inbox_link_v2(base_address, target_address, action),
        LinkTypes::FolderToEmails
        | LinkTypes::EmailToAttachments
        | LinkTypes::EmailToReadReceipts
        | LinkTypes::EmailToDeliveryReceipts
        | LinkTypes::EmailToState
        | LinkTypes::ThreadToEmails
        | LinkTypes::EmailToReplies => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AnchorToPublicEmails => {
            // Public emails anchor - could add rate limiting validation
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

fn validate_inbox_link_v2(
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    action: CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    let inbox_owner = match base_address.into_agent_pub_key() {
        Some(agent) => agent,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "AgentToInboxV2 link base must be an AgentPubKey".into(),
            ));
        }
    };
    let target_action_hash = match target_address.into_action_hash() {
        Some(hash) => hash,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "AgentToInboxV2 target must be an ActionHash".into(),
            ));
        }
    };
    let record = must_get_valid_record(target_action_hash)?;
    let entry = record.entry().as_option().ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "AgentToInboxV2 target record has no entry".into()
        ))
    })?;
    let Entry::App(app_bytes) = entry else {
        return Ok(ValidateCallbackResult::Invalid(
            "AgentToInboxV2 target is not an app entry".into(),
        ));
    };
    let email = match EncryptedEmailV2::try_from(SerializedBytes::from(app_bytes.clone())) {
        Ok(email) => email,
        Err(error) => {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "AgentToInboxV2 target failed to deserialize: {error}"
            )));
        }
    };
    if email.recipient != inbox_owner {
        return Ok(ValidateCallbackResult::Invalid(
            "AgentToInboxV2 base does not match envelope recipient".into(),
        ));
    }
    if email.sender != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "AgentToInboxV2 author does not match envelope sender".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate `AgentToInbox` link creation.
///
/// This closes the spam hole: without these checks, any agent could write a link
/// from ANY base AgentPubKey to ANY entry, polluting other users' inboxes with
/// arbitrary content. Enforces the three invariants that make an inbox trustworthy:
///
/// 1. **Base is an AgentPubKey.** Inbox links address a recipient agent, not an entry.
/// 2. **Target deserializes as `EncryptedEmail`.** Catches attempts to link
///    unrelated entry types into the inbox namespace.
/// 3. **Identity coherence.** The link's base must equal `email.recipient`, and
///    the link's author must equal `email.sender` (which in turn already equals
///    the email's `action.author` via `validate_encrypted_email`). This means
///    Eve cannot deliver to Bob's inbox an envelope Alice authored.
fn validate_inbox_link(
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    action: CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    // 1. Base must be an AgentPubKey (inbox owner).
    let inbox_owner = match base_address.into_agent_pub_key() {
        Some(a) => a,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "AgentToInbox link base must be an AgentPubKey".to_string(),
            ));
        }
    };

    // 2. Target must be an ActionHash pointing at an EncryptedEmail.
    let target_action_hash = match target_address.into_action_hash() {
        Some(h) => h,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "AgentToInbox link target must be an ActionHash".to_string(),
            ));
        }
    };

    let record = must_get_valid_record(target_action_hash)?;
    let entry = record.entry().as_option().ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "AgentToInbox target record has no entry".to_string()
        ))
    })?;

    // Only app entries can be an EncryptedEmail; bail on Agent/Cap/CounterSign.
    let Entry::App(app_bytes) = entry else {
        return Ok(ValidateCallbackResult::Invalid(
            "AgentToInbox target entry is not an app entry".to_string(),
        ));
    };

    let email: EncryptedEmail =
        match EncryptedEmail::try_from(SerializedBytes::from(app_bytes.clone())) {
            Ok(e) => e,
            Err(e) => {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "AgentToInbox target entry failed to deserialize as EncryptedEmail: {}",
                    e
                )));
            }
        };

    // 3. Identity coherence — Eve cannot spam Bob's inbox.
    if email.recipient != inbox_owner {
        return Ok(ValidateCallbackResult::Invalid(
            "AgentToInbox link base does not match envelope recipient".to_string(),
        ));
    }

    if email.sender != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "AgentToInbox link author does not match envelope sender".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_delete_link(
    _link_type: LinkTypes,
    original_action: CreateLink,
    _base_address: AnyLinkableHash,
    _target_address: AnyLinkableHash,
    _tag: LinkTag,
    action: DeleteLink,
) -> ExternResult<ValidateCallbackResult> {
    // Only the original link author can delete the link
    if original_action.author != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the link author can delete a link".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_email() -> EncryptedEmail {
        EncryptedEmail {
            sender: AgentPubKey::from_raw_36(vec![1; 36]),
            recipient: AgentPubKey::from_raw_36(vec![2; 36]),
            encrypted_subject: vec![3; 16],
            encrypted_body: vec![4; 32],
            encrypted_attachments: Vec::new(),
            ephemeral_pubkey: vec![5; 32],
            nonce: [6; 24],
            signature: vec![7; 64],
            crypto_suite: CryptoSuite {
                key_exchange: "x25519".into(),
                symmetric: "aes-256-gcm".into(),
                signature: "ed25519".into(),
            },
            message_id: "message-1".into(),
            in_reply_to: None,
            references: Vec::new(),
            timestamp: Timestamp::from_micros(42),
            priority: EmailPriority::Normal,
            read_receipt_requested: false,
            expires_at: None,
        }
    }

    #[test]
    fn test_email_signing_content_deterministic() {
        let email = test_email();
        assert_eq!(email_signing_content(&email), email_signing_content(&email));
        assert_eq!(SHA256_LEN, 32);
        assert_eq!(ED25519_SIG_LEN, 64);
        assert_eq!(DILITHIUM3_SIG_LEN, 3293);
        assert_eq!(DILITHIUM2_SIG_LEN, 2420);
        assert_eq!(X25519_KEY_LEN, 32);
        assert_eq!(KYBER1024_KEY_LEN, 1568);
        assert_eq!(KYBER768_KEY_LEN, 1088);
        assert_eq!(MAX_CHUNK_SIZE, 10 * 1024 * 1024);
        assert_eq!(MAX_TOTAL_CHUNKS, 1000);
    }

    #[test]
    fn signing_content_binds_routing_crypto_and_metadata() {
        let email = test_email();
        let original = email_signing_content(&email);

        let mut changed = email.clone();
        changed.ephemeral_pubkey[0] ^= 1;
        assert_ne!(original, email_signing_content(&changed));

        let mut changed = email.clone();
        changed.crypto_suite.symmetric = "chacha20-poly1305".into();
        assert_ne!(original, email_signing_content(&changed));

        let mut changed = email;
        changed.read_receipt_requested = true;
        assert_ne!(original, email_signing_content(&changed));
    }

    fn test_email_v2() -> EncryptedEmailV2 {
        EncryptedEmailV2 {
            version: 2,
            cipher_suite:
                mail_leptos_types::protocol::SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65.into(),
            message_id: [1; 32],
            sender: AgentPubKey::from_raw_36(vec![1; 36]),
            recipient: AgentPubKey::from_raw_36(vec![2; 36]),
            sender_mldsa_key_id: [2; 32],
            recipient_hybrid_key_id: [3; 32],
            sender_mldsa_bundle_hash: ActionHash::from_raw_36(vec![10; 36]),
            recipient_bundle_hash: ActionHash::from_raw_36(vec![11; 36]),
            x25519_ephemeral_public_key: [4; 32],
            ml_kem_ciphertext: vec![5; 1088],
            nonce: [6; 12],
            ciphertext: vec![7; 48],
            in_reply_to: None,
            thread_id: None,
            created_at_micros: 42,
            agent_signature: vec![8; ED25519_SIG_LEN],
            ml_dsa_signature: vec![9; ML_DSA_65_SIGNATURE_BYTES],
        }
    }

    /// These structural checks run without any HDI host function, so they
    /// prove real evidence in a plain `cargo test` — no live conductor
    /// needed. The host-dependent checks (agent-signature verification,
    /// sender==author, timestamp skew) are proven instead by the Sweettest
    /// suite, which is the only place a real HDI host is available.
    #[test]
    fn v2_structure_accepts_a_well_formed_entry() {
        assert!(validate_email_v2_structure(&test_email_v2()).is_ok());
    }

    #[test]
    fn v2_structure_rejects_short_ciphertext() {
        let mut email = test_email_v2();
        email.ciphertext = vec![1; 15]; // below the 16-byte AES-GCM tag floor
        let error = validate_email_v2_structure(&email).unwrap_err();
        assert!(error.contains("ciphertext"), "unexpected error: {error}");
    }

    #[test]
    fn v2_structure_rejects_oversized_ciphertext() {
        let mut email = test_email_v2();
        email.ciphertext = vec![1; MAX_ENCRYPTED_BODY_BYTES + 1];
        let error = validate_email_v2_structure(&email).unwrap_err();
        assert!(error.contains("ciphertext"), "unexpected error: {error}");
    }

    #[test]
    fn v2_structure_rejects_wrong_length_ml_dsa_signature() {
        let mut email = test_email_v2();
        email.ml_dsa_signature.pop();
        let error = validate_email_v2_structure(&email).unwrap_err();
        assert!(error.contains("ML-DSA"), "unexpected error: {error}");
    }

    #[test]
    fn v2_structure_rejects_wrong_length_agent_signature() {
        let mut email = test_email_v2();
        email.agent_signature = vec![1; 63];
        let error = validate_email_v2_structure(&email).unwrap_err();
        assert!(
            error.contains("Agent signature"),
            "unexpected error: {error}"
        );
    }

    #[test]
    fn v2_structure_rejects_unknown_version_and_suite() {
        let mut email = test_email_v2();
        email.version = 99;
        let error = validate_email_v2_structure(&email).unwrap_err();
        assert!(error.contains("envelope"), "unexpected error: {error}");

        let mut email = test_email_v2();
        email.cipher_suite = "attacker-chosen-suite".into();
        let error = validate_email_v2_structure(&email).unwrap_err();
        assert!(error.contains("envelope"), "unexpected error: {error}");
    }

    /// `validate_email_v2_structure` is deliberately the host-*independent*
    /// structural check only (lengths, canonical encoding) — a
    /// garbage-but-correctly-sized ML-DSA signature passes here by design.
    /// Cryptographic ML-DSA verification against the sender's historical key
    /// bundle happens in the separate, host-*dependent* `verify_ml_dsa_v2`
    /// (called from `validate_encrypted_email_v2`, proven below) — it was
    /// not merged into this structural check because it needs
    /// `must_get_valid_record`, unavailable without a live HDI host.
    #[test]
    fn v2_structure_does_not_cryptographically_verify_ml_dsa() {
        let mut email = test_email_v2();
        email.ml_dsa_signature = vec![0xAB; ML_DSA_65_SIGNATURE_BYTES]; // garbage, right length
        assert!(validate_email_v2_structure(&email).is_ok());
    }

    /// Real evidence for the ML-DSA gap closed in `verify_ml_dsa_v2`: HDI/WASM
    /// *can* link real ML-DSA-65 verification (see that function's doc
    /// comment) — this proves it end to end using a mocked HDI host
    /// (`hdi::test_utils::set_hdi`) so it runs as a plain, fast `cargo test`
    /// with no live conductor. A real keypair signs a real transcript; the
    /// mock's `must_get_valid_record` returns a `HybridKeyBundleV2` record
    /// authored by the claimed sender, matching `sender_mldsa_key_id`.
    struct MockRecordHdi {
        record: Record,
    }

    impl hdi::hdi::HdiT for MockRecordHdi {
        fn must_get_valid_record(&self, _: MustGetValidRecordInput) -> ExternResult<Record> {
            Ok(self.record.clone())
        }
        fn verify_signature(&self, _: VerifySignature) -> ExternResult<bool> {
            unimplemented!("not exercised by verify_ml_dsa_v2")
        }
        fn must_get_entry(&self, _: MustGetEntryInput) -> ExternResult<EntryHashed> {
            unimplemented!("not exercised by verify_ml_dsa_v2")
        }
        fn must_get_action(&self, _: MustGetActionInput) -> ExternResult<SignedActionHashed> {
            unimplemented!("not exercised by verify_ml_dsa_v2")
        }
        fn must_get_agent_activity(
            &self,
            _: MustGetAgentActivityInput,
        ) -> ExternResult<Vec<RegisterAgentActivity>> {
            unimplemented!("not exercised by verify_ml_dsa_v2")
        }
        fn dna_info(&self, _: ()) -> ExternResult<DnaInfo> {
            unimplemented!("not exercised by verify_ml_dsa_v2")
        }
        fn zome_info(&self, _: ()) -> ExternResult<ZomeInfo> {
            unimplemented!("not exercised by verify_ml_dsa_v2")
        }
        fn trace(&self, _: TraceMsg) -> ExternResult<()> {
            unimplemented!("not exercised by verify_ml_dsa_v2")
        }
        fn x_salsa20_poly1305_decrypt(
            &self,
            _: XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by verify_ml_dsa_v2")
        }
        fn x_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: X25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by verify_ml_dsa_v2")
        }
        fn ed_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: Ed25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<XSalsa20Poly1305Data> {
            unimplemented!("not exercised by verify_ml_dsa_v2")
        }
    }

    /// Builds a `Record` wrapping a `HybridKeyBundleV2` entry, authored by
    /// `author`, suitable for the mock's `must_get_valid_record` to return.
    fn bundle_record(author: AgentPubKey, bundle: &HybridKeyBundleV2) -> Record {
        let entry = Entry::App(
            AppEntryBytes::try_from(SerializedBytes::try_from(bundle.clone()).unwrap()).unwrap(),
        );
        let entry_hash = EntryHash::from_raw_36(vec![9; 36]);
        let action = Action::Create(Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash,
            weight: Default::default(),
        });
        let signed_action = SignedActionHashed::new_unchecked(action, Signature([0; 64]));
        Record::new(signed_action, Some(entry))
    }

    /// Builds a `Record` wrapping an `EncryptedEmail` entry, authored by `author`,
    /// suitable for the mock's `must_get_valid_record` to return.
    fn email_record(author: AgentPubKey, email: &EncryptedEmail) -> Record {
        let entry = Entry::App(
            AppEntryBytes::try_from(SerializedBytes::try_from(email.clone()).unwrap()).unwrap(),
        );
        let entry_hash = EntryHash::from_raw_36(vec![19; 36]);
        let action = Action::Create(Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![18; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash,
            weight: Default::default(),
        });
        let signed_action = SignedActionHashed::new_unchecked(action, Signature([0; 64]));
        Record::new(signed_action, Some(entry))
    }

    fn test_attachment(email_hash: ActionHash) -> EncryptedAttachment {
        EncryptedAttachment {
            email_hash,
            encrypted_filename: vec![1; 8],
            encrypted_mime_type: vec![2; 8],
            encrypted_content: vec![3; 16],
            chunk_index: 0,
            total_chunks: 1,
            content_hash: vec![4; 32],
            nonce: [5; 24],
        }
    }

    /// Proves the P0 author-binding fix: an EncryptedAttachment whose `email_hash`
    /// resolves to an email sent by someone other than the attachment's own
    /// committer is rejected (previously any agent could attach arbitrary content
    /// to any email in the system).
    #[test]
    fn attachment_citing_someone_elses_email_is_rejected() {
        let real_sender = AgentPubKey::from_raw_36(vec![1; 36]);
        let impostor = AgentPubKey::from_raw_36(vec![99; 36]);
        let mut email = test_email();
        email.sender = real_sender.clone();
        let email_hash = ActionHash::from_raw_36(vec![20; 36]);

        hdi::hdi::set_hdi(MockRecordHdi {
            record: email_record(real_sender, &email),
        });

        let attachment = test_attachment(email_hash);
        let action = Create {
            author: impostor,
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
        };
        let result = validate_attachment(&attachment, &action).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn attachment_citing_its_own_email_is_accepted() {
        let sender = AgentPubKey::from_raw_36(vec![1; 36]);
        let mut email = test_email();
        email.sender = sender.clone();
        let email_hash = ActionHash::from_raw_36(vec![20; 36]);

        hdi::hdi::set_hdi(MockRecordHdi {
            record: email_record(sender.clone(), &email),
        });

        let attachment = test_attachment(email_hash);
        let action = Create {
            author: sender,
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
        };
        let result = validate_attachment(&attachment, &action).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    fn test_bundle(ml_dsa_verifying_key: Vec<u8>) -> HybridKeyBundleV2 {
        HybridKeyBundleV2 {
            version: 2,
            suite: mail_leptos_types::protocol::SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65
                .into(),
            key_id: [0; 32],
            x25519_public_key: [1; 32],
            ml_kem_768_public_key: vec![2; 1184],
            ml_dsa_65_public_key: ml_dsa_verifying_key,
            state: HybridKeyStateV2::Active,
            created_at: 0,
            expires_at: u64::MAX,
            agent_signature: vec![3; 64],
        }
    }

    #[test]
    fn verify_ml_dsa_v2_accepts_a_genuinely_valid_signature() {
        use ml_dsa::{Generate, Keypair};
        let signing_key = ml_dsa::SigningKey::<ml_dsa::MlDsa65>::generate();
        let verifying_key = signing_key.verifying_key();
        let vk_bytes = {
            use ml_dsa::common::KeyExport;
            verifying_key.to_bytes().to_vec()
        };

        let transcript = b"pulse-v2-test-transcript".to_vec();
        let ml_dsa_signature = {
            use ml_dsa::signature::{SignatureEncoding, Signer};
            signing_key.sign(&transcript).to_bytes().to_vec()
        };

        let author = AgentPubKey::from_raw_36(vec![7; 36]);
        let mut bundle = test_bundle(vk_bytes);
        bundle.key_id = hybrid_key_id(&bundle);

        let mut email = test_email_v2();
        email.sender = author.clone();
        email.sender_mldsa_key_id = bundle.key_id;
        email.ml_dsa_signature = ml_dsa_signature;

        hdi::hdi::set_hdi(MockRecordHdi {
            record: bundle_record(author, &bundle),
        });

        assert_eq!(
            verify_ml_dsa_v2(&email, &transcript).unwrap(),
            Ok(()),
            "a genuinely valid ML-DSA signature over the exact transcript must verify"
        );
    }

    #[test]
    fn verify_ml_dsa_v2_rejects_a_tampered_signature() {
        use ml_dsa::{Generate, Keypair};
        let signing_key = ml_dsa::SigningKey::<ml_dsa::MlDsa65>::generate();
        let verifying_key = signing_key.verifying_key();
        let vk_bytes = {
            use ml_dsa::common::KeyExport;
            verifying_key.to_bytes().to_vec()
        };

        let transcript = b"pulse-v2-test-transcript".to_vec();
        let mut ml_dsa_signature = {
            use ml_dsa::signature::{SignatureEncoding, Signer};
            signing_key.sign(&transcript).to_bytes().to_vec()
        };
        ml_dsa_signature[0] ^= 1; // tamper with one bit of an otherwise-real signature

        let author = AgentPubKey::from_raw_36(vec![7; 36]);
        let mut bundle = test_bundle(vk_bytes);
        bundle.key_id = hybrid_key_id(&bundle);

        let mut email = test_email_v2();
        email.sender = author.clone();
        email.sender_mldsa_key_id = bundle.key_id;
        email.ml_dsa_signature = ml_dsa_signature;

        hdi::hdi::set_hdi(MockRecordHdi {
            record: bundle_record(author, &bundle),
        });

        assert!(
            verify_ml_dsa_v2(&email, &transcript).unwrap().is_err(),
            "a tampered ML-DSA signature must be rejected"
        );
    }

    #[test]
    fn verify_ml_dsa_v2_rejects_a_bundle_from_the_wrong_author() {
        use ml_dsa::{Generate, Keypair};
        let signing_key = ml_dsa::SigningKey::<ml_dsa::MlDsa65>::generate();
        let verifying_key = signing_key.verifying_key();
        let vk_bytes = {
            use ml_dsa::common::KeyExport;
            verifying_key.to_bytes().to_vec()
        };

        let transcript = b"pulse-v2-test-transcript".to_vec();
        let ml_dsa_signature = {
            use ml_dsa::signature::{SignatureEncoding, Signer};
            signing_key.sign(&transcript).to_bytes().to_vec()
        };

        let real_author = AgentPubKey::from_raw_36(vec![7; 36]);
        let claimed_sender = AgentPubKey::from_raw_36(vec![77; 36]);
        let mut bundle = test_bundle(vk_bytes);
        bundle.key_id = hybrid_key_id(&bundle);

        let mut email = test_email_v2();
        email.sender = claimed_sender;
        email.sender_mldsa_key_id = bundle.key_id;
        email.ml_dsa_signature = ml_dsa_signature;

        // The bundle record is authored by `real_author`, not `email.sender` —
        // a sender can't borrow someone else's key bundle by pointing at it.
        hdi::hdi::set_hdi(MockRecordHdi {
            record: bundle_record(real_author, &bundle),
        });

        assert!(
            verify_ml_dsa_v2(&email, &transcript).unwrap().is_err(),
            "a bundle authored by someone else must be rejected"
        );
    }

    /// Real evidence for the recipient-side half of the ML-DSA gap: a
    /// message to a recipient whose key bundle is genuinely `Active` is
    /// accepted.
    #[test]
    fn verify_recipient_key_state_accepts_an_active_recipient_bundle() {
        let recipient = AgentPubKey::from_raw_36(vec![2; 36]);
        let mut bundle = test_bundle(vec![0; 1952]); // ML-DSA-65 pubkey size; unused here
        bundle.key_id = hybrid_key_id(&bundle);

        let mut email = test_email_v2();
        email.recipient = recipient.clone();
        email.recipient_hybrid_key_id = bundle.key_id;

        hdi::hdi::set_hdi(MockRecordHdi {
            record: bundle_record(recipient, &bundle),
        });

        assert_eq!(
            verify_recipient_key_state(&email).unwrap(),
            Ok(()),
            "a recipient bundle in the Active state must be accepted"
        );
    }

    #[test]
    fn verify_recipient_key_state_rejects_a_revoked_recipient_bundle() {
        let recipient = AgentPubKey::from_raw_36(vec![2; 36]);
        let mut bundle = test_bundle(vec![0; 1952]);
        bundle.state = HybridKeyStateV2::RevokedCompromised;
        bundle.key_id = hybrid_key_id(&bundle);

        let mut email = test_email_v2();
        email.recipient = recipient.clone();
        email.recipient_hybrid_key_id = bundle.key_id;

        hdi::hdi::set_hdi(MockRecordHdi {
            record: bundle_record(recipient, &bundle),
        });

        assert!(
            verify_recipient_key_state(&email).unwrap().is_err(),
            "a message to a recipient whose key bundle is revoked must be rejected"
        );
    }

    #[test]
    fn verify_recipient_key_state_rejects_a_bundle_from_the_wrong_author() {
        let real_owner = AgentPubKey::from_raw_36(vec![2; 36]);
        let claimed_recipient = AgentPubKey::from_raw_36(vec![22; 36]);
        let mut bundle = test_bundle(vec![0; 1952]);
        bundle.key_id = hybrid_key_id(&bundle);

        let mut email = test_email_v2();
        email.recipient = claimed_recipient;
        email.recipient_hybrid_key_id = bundle.key_id;

        // The bundle record is authored by `real_owner`, not `email.recipient` —
        // a sender can't point `recipient_bundle_hash` at someone else's bundle.
        hdi::hdi::set_hdi(MockRecordHdi {
            record: bundle_record(real_owner, &bundle),
        });

        assert!(
            verify_recipient_key_state(&email).unwrap().is_err(),
            "a bundle authored by someone other than the claimed recipient must be rejected"
        );
    }
}
