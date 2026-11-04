// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Capabilities Integrity Zome
//!
//! Fine-grained access control for Mycelix Mail using Holochain capabilities.
//! Enables shared mailboxes, delegated access, and time-limited permissions.

use hdi::prelude::*;

/// Capability grant for mailbox access
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MailboxCapability {
    /// Unique identifier for this capability
    pub id: String,
    /// Grantor (mailbox owner)
    pub grantor: AgentPubKey,
    /// Grantee (who receives access)
    pub grantee: AgentPubKey,
    /// Type of access granted
    pub access_type: MailboxAccessType,
    /// Specific permissions
    pub permissions: MailboxPermissions,
    /// Optional resource restrictions
    pub restrictions: Option<AccessRestrictions>,
    /// When capability was granted
    pub granted_at: Timestamp,
    /// Expiration time
    pub expires_at: Option<Timestamp>,
    /// Whether capability has been revoked
    pub revoked: bool,
    /// Revocation reason if revoked
    pub revocation_reason: Option<String>,
    /// Action hash of the local Holochain capability grant that authorizes remote calls.
    /// Absent only on records created before this binding existed; those cannot be
    /// declared conductor-revoked without a separate audited migration.
    #[serde(default)]
    pub system_grant_action_hash: Option<ActionHash>,
    /// SHA-256 fingerprint of the grant secret. The secret itself must never be stored
    /// in this public application entry; it belongs in the recipient's private CapClaim.
    pub secret_hash: Vec<u8>,
}

/// Type of mailbox access
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum MailboxAccessType {
    /// Full access (owner equivalent)
    FullAccess,
    /// Read-only access
    ReadOnly,
    /// Send on behalf of
    SendAs,
    /// Manage specific folder
    FolderAccess { folder_hash: ActionHash },
    /// Access to specific thread
    ThreadAccess { thread_id: String },
    /// Temporary out-of-office delegation
    OutOfOffice,
    /// Admin access for organization
    OrganizationAdmin,
    /// Custom access type
    Custom(String),
}

/// Granular permissions
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Default)]
pub struct MailboxPermissions {
    /// Can read emails
    pub can_read: bool,
    /// Can send emails
    pub can_send: bool,
    /// Can delete emails
    pub can_delete: bool,
    /// Can move emails between folders
    pub can_move: bool,
    /// Can create folders
    pub can_create_folders: bool,
    /// Can manage labels
    pub can_manage_labels: bool,
    /// Can view attachments
    pub can_view_attachments: bool,
    /// Can download attachments
    pub can_download_attachments: bool,
    /// Can manage filters/rules
    pub can_manage_rules: bool,
    /// Can grant sub-capabilities
    pub can_delegate: bool,
    /// Can modify settings
    pub can_modify_settings: bool,
    /// Can view trust scores
    pub can_view_trust: bool,
    /// Can modify trust attestations
    pub can_modify_trust: bool,
}

/// Restrictions on access
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AccessRestrictions {
    /// Only specific folders
    pub folder_whitelist: Option<Vec<ActionHash>>,
    /// Excluded folders
    pub folder_blacklist: Option<Vec<ActionHash>>,
    /// Only emails from specific senders
    pub sender_whitelist: Option<Vec<String>>,
    /// Maximum emails accessible
    pub max_emails: Option<u32>,
    /// Only emails after this date
    pub date_from: Option<Timestamp>,
    /// Only emails before this date
    pub date_to: Option<Timestamp>,
    /// IP/network restrictions (for enterprise)
    pub network_restrictions: Option<Vec<String>>,
    /// Require 2FA for access
    pub require_2fa: bool,
    /// Audit all access
    pub audit_required: bool,
}

/// Shared mailbox configuration
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SharedMailbox {
    /// Unique identifier
    pub id: String,
    /// Display name
    pub name: String,
    /// Owner agent
    pub owner: AgentPubKey,
    /// Email address for this mailbox
    pub email_address: String,
    /// Members with access
    pub members: Vec<SharedMailboxMember>,
    /// When created
    pub created_at: Timestamp,
    /// Whether active
    pub is_active: bool,
    /// Mailbox settings
    pub settings: SharedMailboxSettings,
}

/// Member of a shared mailbox
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SharedMailboxMember {
    /// Member agent
    pub agent: AgentPubKey,
    /// Their role
    pub role: SharedMailboxRole,
    /// Their specific permissions
    pub permissions: MailboxPermissions,
    /// When added
    pub added_at: Timestamp,
    /// Who added them
    pub added_by: AgentPubKey,
}

/// Roles in shared mailbox
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum SharedMailboxRole {
    Owner,
    Admin,
    Editor,
    Contributor,
    Viewer,
}

/// Settings for shared mailbox
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Default)]
pub struct SharedMailboxSettings {
    /// Send notifications on new mail
    pub notify_all_on_new_mail: bool,
    /// Allow members to send as mailbox
    pub allow_send_as: bool,
    /// Require approval for external sends
    pub require_send_approval: bool,
    /// Auto-assign emails to members
    pub auto_assign: bool,
    /// Assignment strategy
    pub assignment_strategy: Option<AssignmentStrategy>,
}

/// Strategy for auto-assigning emails
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum AssignmentStrategy {
    RoundRobin,
    LeastBusy,
    ByKeyword { rules: Vec<(String, AgentPubKey)> },
    BySender { rules: Vec<(String, AgentPubKey)> },
    Random,
}

/// Capability audit log entry
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CapabilityAuditLog {
    /// Capability that was used
    pub capability_id: String,
    /// Who used it
    pub actor: AgentPubKey,
    /// What action was taken
    pub action: AuditAction,
    /// Resource affected
    pub resource: Option<ActionHash>,
    /// Timestamp
    pub timestamp: Timestamp,
    /// Success or failure
    pub success: bool,
    /// Error message if failed
    pub error: Option<String>,
    /// Additional context
    pub context: Option<String>,
}

/// Actions that can be audited
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum AuditAction {
    ReadEmail,
    SendEmail,
    DeleteEmail,
    MoveEmail,
    CreateFolder,
    DeleteFolder,
    ModifySettings,
    GrantCapability,
    RevokeCapability,
    AccessAttachment,
    ModifyTrust,
    ExportData,
    Login,
    Logout,
    Custom(String),
}

/// Delegation chain tracking
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DelegationChain {
    /// Original grantor
    pub root_grantor: AgentPubKey,
    /// Chain of delegations
    pub chain: Vec<DelegationHop>,
    /// Current capability hash
    pub current_capability: ActionHash,
    /// Maximum chain length allowed
    pub max_depth: u8,
}

/// Single hop in delegation chain
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DelegationHop {
    pub from: AgentPubKey,
    pub to: AgentPubKey,
    pub capability_hash: ActionHash,
    pub timestamp: Timestamp,
}

/// Link types
#[hdk_link_types]
pub enum LinkTypes {
    /// Agent -> capabilities they've granted
    AgentToGrantedCapabilities,
    /// Agent -> capabilities they've received
    AgentToReceivedCapabilities,
    /// Capability -> audit logs
    CapabilityToAuditLogs,
    /// Shared mailbox -> members
    SharedMailboxToMembers,
    /// Agent -> shared mailboxes they own
    AgentToOwnedSharedMailboxes,
    /// Agent -> shared mailboxes they're member of
    AgentToMemberSharedMailboxes,
    /// Capability -> delegation chain
    CapabilityToDelegationChain,
}

/// Entry types
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(required_validations = 2)]
    MailboxCapability(MailboxCapability),
    #[entry_type(required_validations = 2)]
    SharedMailbox(SharedMailbox),
    #[entry_type(required_validations = 1)]
    CapabilityAuditLog(CapabilityAuditLog),
    #[entry_type(required_validations = 2)]
    DelegationChain(DelegationChain),
}

// ==================== VALIDATION ====================

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::CreateEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => {
                validate_create_entry(app_entry, action.into())
            }
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => validate_update_entry(app_entry, action),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::CreateRecord(record) => match record {
            OpRecord::CreateEntry { app_entry, action } => {
                validate_create_entry(app_entry, action.into())
            }
            OpRecord::UpdateEntry { app_entry, action } => validate_update_entry(app_entry, action),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::Update(OpUpdate::Entry { app_entry, action }) => {
            validate_update_entry(app_entry, action)
        }
        FlatOp::Update(_) => Ok(ValidateCallbackResult::Invalid(
            "Unsupported capability update variant".to_string(),
        )),
        FlatOp::Delete(_) => Ok(ValidateCallbackResult::Invalid(
            "Capability entries cannot be deleted".to_string(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_entry(
    entry: EntryTypes,
    action: TypedAction<EntryCreationData>,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::MailboxCapability(cap) => validate_capability(&cap, &action),
        EntryTypes::SharedMailbox(mailbox) => validate_shared_mailbox(&mailbox, &action),
        EntryTypes::CapabilityAuditLog(log) => validate_audit_log(&log, &action),
        EntryTypes::DelegationChain(chain) => validate_delegation_chain(&chain, &action),
    }
}

fn validate_update_entry(
    entry: EntryTypes,
    action: TypedAction<UpdateData>,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        // Capabilities can only be updated by grantor (to revoke)
        EntryTypes::MailboxCapability(cap) => {
            if cap.grantor != *action.author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only grantor can update capability".to_string(),
                ));
            }

            // Resolve the source entry and permit exactly one monotonic transition:
            // active -> revoked. The grant binding, grantee, permissions, restrictions,
            // timestamps, and secret fingerprint are immutable; a revoked capability
            // can never be reactivated or edited.
            let original_record = must_get_valid_record(action.original_action_address.clone())?;
            let original: MailboxCapability = original_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Original capability update points to another entry type".to_string()
                )))?;

            if original.grantor != *action.author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Original capability must belong to the updating grantor".to_string(),
                ));
            }

            if cap.id != original.id
                || cap.grantor != original.grantor
                || cap.grantee != original.grantee
                || cap.access_type != original.access_type
                || cap.permissions != original.permissions
                || cap.restrictions != original.restrictions
                || cap.granted_at != original.granted_at
                || cap.expires_at != original.expires_at
                || cap.system_grant_action_hash != original.system_grant_action_hash
                || cap.secret_hash != original.secret_hash
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Capability authority fields are immutable".to_string(),
                ));
            }

            if original.revoked || !cap.revoked {
                return Ok(ValidateCallbackResult::Invalid(
                    "Capability updates may only transition active -> revoked once".to_string(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        // Direct SharedMailbox updates are currently limited to the original owner.
        // Delegated Admin transitions require a controller-signed execution lease and
        // a separate immutable proof; this update shape does not carry that proof.
        EntryTypes::SharedMailbox(mailbox) => {
            let original_record = must_get_valid_record(action.original_action_address.clone())?;
            let original: SharedMailbox = original_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Original mailbox update points to another entry type".to_string()
                )))?;

            if original.owner != *action.author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Direct mailbox member transitions require the original owner; delegated Admin changes require an execution lease".to_string(),
                ));
            }

            if !is_valid_owner_member_append(&original, &mailbox, action.author()) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Mailbox update must be exactly one owner-authored member append with all existing state unchanged".to_string(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        // Audit logs are immutable
        EntryTypes::CapabilityAuditLog(_) => Ok(ValidateCallbackResult::Invalid(
            "Audit logs cannot be modified".to_string(),
        )),
        _ => Ok(ValidateCallbackResult::Invalid(
            "Unsupported capability entry update".to_string(),
        )),
    }
}

fn validate_capability(
    cap: &MailboxCapability,
    action: &TypedAction<EntryCreationData>,
) -> ExternResult<ValidateCallbackResult> {
    // Grantor must be author
    if cap.grantor != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Grantor must match author".to_string(),
        ));
    }

    // Can't grant to self
    if cap.grantor == cap.grantee {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot grant capability to self".to_string(),
        ));
    }

    // The public field is specifically a SHA-256 fingerprint, not an arbitrary
    // non-empty byte string. Enforce the digest width at the integrity boundary.
    if !is_sha256_fingerprint(&cap.secret_hash) {
        return Ok(ValidateCallbackResult::Invalid(
            "Capability secret fingerprint must be exactly 32 bytes (SHA-256)".to_string(),
        ));
    }

    // If has expiration, must be in future
    if let Some(expires) = cap.expires_at {
        if expires <= cap.granted_at {
            return Ok(ValidateCallbackResult::Invalid(
                "Expiration must be after grant time".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate the only SharedMailbox update shape currently exposed by Pulse:
/// one new member appended by the original owner. The original entry is fetched
/// separately by the caller; authority is never inferred from proposed membership.
fn is_valid_owner_member_append(
    original: &SharedMailbox,
    proposed: &SharedMailbox,
    author: &AgentPubKey,
) -> bool {
    if original.owner != *author
        || proposed.id != original.id
        || proposed.name != original.name
        || proposed.owner != original.owner
        || proposed.email_address != original.email_address
        || proposed.created_at != original.created_at
        || proposed.is_active != original.is_active
        || proposed.settings != original.settings
        || original.members.len().checked_add(1) != Some(proposed.members.len())
        || !proposed.members.starts_with(&original.members)
    {
        return false;
    }

    let Some(added_member) = proposed.members.last() else {
        return false;
    };

    added_member.added_by == *author
        && !matches!(added_member.role, SharedMailboxRole::Owner)
        && !original
            .members
            .iter()
            .any(|member| member.agent == added_member.agent)
}

fn validate_shared_mailbox(
    mailbox: &SharedMailbox,
    action: &TypedAction<EntryCreationData>,
) -> ExternResult<ValidateCallbackResult> {
    // Owner must be author
    if mailbox.owner != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Owner must match author".to_string(),
        ));
    }

    // Must have name
    if mailbox.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Shared mailbox must have name".to_string(),
        ));
    }

    // Must have email address
    if mailbox.email_address.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Shared mailbox must have email address".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_audit_log(
    _log: &CapabilityAuditLog,
    _action: &TypedAction<EntryCreationData>,
) -> ExternResult<ValidateCallbackResult> {
    // Audit logs have minimal validation - they're append-only
    Ok(ValidateCallbackResult::Valid)
}

fn validate_delegation_chain(
    chain: &DelegationChain,
    _action: &TypedAction<EntryCreationData>,
) -> ExternResult<ValidateCallbackResult> {
    // Chain must not exceed max depth
    if chain.chain.len() > chain.max_depth as usize {
        return Ok(ValidateCallbackResult::Invalid(
            "Delegation chain exceeds maximum depth".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod shared_mailbox_transition_tests {
    use super::*;

    fn agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_32(vec![byte; 32])
    }

    fn member(
        agent: AgentPubKey,
        role: SharedMailboxRole,
        added_by: AgentPubKey,
    ) -> SharedMailboxMember {
        SharedMailboxMember {
            agent,
            role,
            permissions: MailboxPermissions::default(),
            added_at: Timestamp::from_micros(1),
            added_by,
        }
    }

    fn mailbox(owner: AgentPubKey) -> SharedMailbox {
        SharedMailbox {
            id: "mailbox-1".to_string(),
            name: "Operations".to_string(),
            owner: owner.clone(),
            email_address: "ops@example.test".to_string(),
            members: vec![member(
                owner.clone(),
                SharedMailboxRole::Owner,
                owner.clone(),
            )],
            created_at: Timestamp::from_micros(0),
            is_active: true,
            settings: SharedMailboxSettings::default(),
        }
    }

    #[test]
    fn original_owner_can_append_one_new_member() {
        let owner = agent(1);
        let mut proposed = mailbox(owner.clone());
        proposed
            .members
            .push(member(agent(2), SharedMailboxRole::Viewer, owner.clone()));
        assert!(is_valid_owner_member_append(
            &mailbox(owner.clone()),
            &proposed,
            &owner
        ));
    }

    #[test]
    fn proposed_admin_membership_cannot_self_authorize() {
        let owner = agent(1);
        let attacker = agent(2);
        let original = mailbox(owner.clone());
        let mut proposed = original.clone();
        proposed.members.push(member(
            attacker.clone(),
            SharedMailboxRole::Admin,
            attacker.clone(),
        ));
        assert!(!is_valid_owner_member_append(
            &original, &proposed, &attacker
        ));
    }

    #[test]
    fn existing_admin_cannot_use_owner_only_direct_transition() {
        let owner = agent(1);
        let admin = agent(2);
        let mut original = mailbox(owner.clone());
        original.members.push(member(
            admin.clone(),
            SharedMailboxRole::Admin,
            owner.clone(),
        ));
        let mut proposed = original.clone();
        proposed
            .members
            .push(member(agent(3), SharedMailboxRole::Viewer, admin.clone()));
        assert!(!is_valid_owner_member_append(&original, &proposed, &admin));
    }

    #[test]
    fn member_append_cannot_smuggle_name_or_settings_mutation() {
        let owner = agent(1);
        let mut proposed = mailbox(owner.clone());
        proposed
            .members
            .push(member(agent(2), SharedMailboxRole::Viewer, owner.clone()));
        proposed.name = "Attacker controlled".to_string();
        assert!(!is_valid_owner_member_append(
            &mailbox(owner.clone()),
            &proposed,
            &owner
        ));
        proposed.name = "Operations".to_string();
        proposed.settings.allow_send_as = true;
        assert!(!is_valid_owner_member_append(
            &mailbox(owner.clone()),
            &proposed,
            &owner
        ));
    }

    #[test]
    fn member_append_cannot_rewrite_existing_member_or_duplicate_agent() {
        let owner = agent(1);
        let admin = agent(2);
        let original_member = member(admin.clone(), SharedMailboxRole::Admin, owner.clone());
        let mut original = mailbox(owner.clone());
        original.members.push(original_member);
        let mut rewritten = original.clone();
        rewritten.members[1].role = SharedMailboxRole::Owner;
        rewritten
            .members
            .push(member(agent(3), SharedMailboxRole::Viewer, owner.clone()));
        assert!(!is_valid_owner_member_append(&original, &rewritten, &owner));

        let mut duplicate = original.clone();
        duplicate.members.push(member(
            admin.clone(),
            SharedMailboxRole::Viewer,
            owner.clone(),
        ));
        assert!(!is_valid_owner_member_append(&original, &duplicate, &owner));
    }

    #[test]
    fn direct_member_append_cannot_assign_another_owner_role() {
        let owner = agent(1);
        let mut proposed = mailbox(owner.clone());
        proposed
            .members
            .push(member(agent(2), SharedMailboxRole::Owner, owner.clone()));
        assert!(!is_valid_owner_member_append(
            &mailbox(owner.clone()),
            &proposed,
            &owner
        ));
    }
}

const SHA256_FINGERPRINT_LEN: usize = 32;

fn is_sha256_fingerprint(secret_hash: &[u8]) -> bool {
    secret_hash.len() == SHA256_FINGERPRINT_LEN
}

#[cfg(test)]
mod capability_fingerprint_tests {
    use super::is_sha256_fingerprint;

    #[test]
    fn capability_secret_fingerprint_must_be_exactly_sha256_width() {
        assert!(!is_sha256_fingerprint(&[]));
        assert!(!is_sha256_fingerprint(&[7; 31]));
        assert!(is_sha256_fingerprint(&[7; 32]));
        assert!(!is_sha256_fingerprint(&[7; 33]));
    }
}
