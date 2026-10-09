// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
#![deny(unsafe_code)]
//! Capabilities Coordinator Zome
//!
//! Manages fine-grained access control, shared mailboxes, and delegation.

use hdk::prelude::*;
use mail_capabilities_integrity::*;
use sha2::{Digest, Sha256};
use std::collections::{BTreeSet, HashSet};

/// Signal types for capability events
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "type", content = "data")]
pub enum CapabilitySignal {
    /// New capability granted
    CapabilityGranted {
        capability_hash: ActionHash,
        grantor: AgentPubKey,
        access_type: MailboxAccessType,
    },
    /// Capability revoked
    CapabilityRevoked {
        capability_id: String,
        grantor: AgentPubKey,
        reason: Option<String>,
    },
    /// Added to shared mailbox
    AddedToSharedMailbox {
        mailbox_hash: ActionHash,
        mailbox_name: String,
        role: SharedMailboxRole,
    },
    /// New email in shared mailbox
    SharedMailboxNewEmail {
        mailbox_hash: ActionHash,
        email_hash: ActionHash,
    },
    /// Capability used (audit notification)
    CapabilityUsed {
        capability_id: String,
        action: AuditAction,
    },
}
/// Private wire envelope used only to deliver a newly issued capability secret to
/// the intended grantee. The secret is deliberately excluded from CapabilitySignal,
/// which is emitted to local UI listeners.
#[derive(Serialize, Deserialize, Debug, Clone)]
struct CapabilityGrantDelivery {
    capability_hash: ActionHash,
    capability_id: String,
    grantor: AgentPubKey,
    access_type: MailboxAccessType,
    secret: CapSecret,
}

// ==================== GRANT CAPABILITY ====================

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct GrantCapabilityInput {
    pub grantee: AgentPubKey,
    pub access_type: MailboxAccessType,
    pub permissions: MailboxPermissions,
    pub restrictions: Option<AccessRestrictions>,
    pub expires_at: Option<Timestamp>,
}

/// Grant a new capability to another agent
#[hdk_extern]
pub fn grant_capability(input: GrantCapabilityInput) -> ExternResult<ActionHash> {
    let my_agent = agent_info()?.agent_initial_pubkey;
    let now = sys_time()?;

    // A Holochain CapGrant has no native expires_at field. Issuing an expiring
    // application record while leaving the conductor grant alive would be a
    // security lie, so refuse expiry until a qualified revocation scheduler exists.
    if input.expires_at.is_some() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Expiring remote capabilities are disabled until conductor-level timed revocation is qualified".to_string(),
        )));
    }

    // Holochain's host-generated 512-bit secret is the only supported source of
    // capability secrets. Never persist it in the application/DHT entry.
    let id = format!("cap_{}_{}", my_agent, now.as_micros());
    let secret = generate_cap_secret()?;
    let secret_hash = Sha256::digest(secret.as_ref()).to_vec();
    let functions = determine_granted_functions(
        &input.access_type,
        &input.permissions,
        input.restrictions.as_ref(),
    )?;

    // The private grant's exact action hash is required to revoke conductor-level
    // authorization later. The entire zome call is atomic, so later failures roll
    // back this grant rather than leaving an untracked authorization behind.
    let system_grant_action_hash = create_cap_grant(CapGrantEntry {
        tag: id.clone(),
        access: CapAccess::Assigned {
            secret: secret.clone(),
            assignees: BTreeSet::from([input.grantee.clone()]),
        },
        functions,
    })?;

    let capability = MailboxCapability {
        id: id.clone(),
        grantor: my_agent.clone(),
        grantee: input.grantee.clone(),
        access_type: input.access_type.clone(),
        permissions: input.permissions,
        restrictions: input.restrictions,
        granted_at: now,
        expires_at: input.expires_at,
        revoked: false,
        revocation_reason: None,
        system_grant_action_hash: Some(system_grant_action_hash),
        secret_hash,
    };

    let cap_hash = create_entry(EntryTypes::MailboxCapability(capability))?;

    // Link from grantor
    create_link(
        my_agent.clone(),
        cap_hash.clone(),
        LinkTypes::AgentToGrantedCapabilities,
        LinkTag::new(format!("to:{}", input.grantee)),
    )?;

    // Link to grantee
    create_link(
        input.grantee.clone(),
        cap_hash.clone(),
        LinkTypes::AgentToReceivedCapabilities,
        LinkTag::new(format!("from:{}", my_agent)),
    )?;

    // Do not send the secret from this transaction: its local grant/entry actions
    // have not committed yet. The caller must invoke deliver_capability_grant after
    // this call succeeds; that second call performs an acknowledged remote handoff.

    // Audit log
    log_capability_action(&cap_hash, AuditAction::GrantCapability, true, None)?;

    Ok(cap_hash)
}

fn determine_granted_functions(
    access_type: &MailboxAccessType,
    permissions: &MailboxPermissions,
    restrictions: Option<&AccessRestrictions>,
) -> ExternResult<GrantedFunctions> {
    // This grant targets mail_messages, not the current mail_capabilities zome.
    // Never use GrantedFunctions::All: a capability must not accidentally expose
    // unrelated identity, key, trust, federation, backup, or capability functions.
    const MESSAGE_ZOME: &str = "mail_messages";

    if restrictions.is_some_and(|r| {
        r.folder_whitelist.is_some()
            || r.folder_blacklist.is_some()
            || r.sender_whitelist.is_some()
            || r.max_emails.is_some()
            || r.date_from.is_some()
            || r.date_to.is_some()
            || r.network_restrictions.is_some()
            || r.require_2fa
            || r.audit_required
    }) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Capability restrictions are not yet enforced by remote message entrypoints; refusing a broader grant".to_string(),
        )));
    }

    if permissions.can_manage_labels
        || permissions.can_manage_rules
        || permissions.can_delegate
        || permissions.can_modify_settings
        || permissions.can_view_trust
        || permissions.can_modify_trust
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Requested permission is not backed by a capability-scoped remote entrypoint".to_string(),
        )));
    }

    if permissions.can_view_attachments != permissions.can_download_attachments {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Attachment access currently requires view and download permissions to agree".to_string(),
        )));
    }
    if permissions.can_view_attachments && !permissions.can_read {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Attachment access requires read permission".to_string(),
        )));
    }

    // The current remote-mail API has no general message-deletion entrypoint.
    // Do not pretend delete_draft implements the broader can_delete contract.
    if permissions.can_delete {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "can_delete is disabled for delegated capabilities until a scoped delete entrypoint exists".to_string(),
        )));
    }

    let (read_allowed, write_allowed) = match access_type {
        MailboxAccessType::FullAccess => (true, true),
        MailboxAccessType::ReadOnly => {
            if permissions.can_send
                || permissions.can_delete
                || permissions.can_move
                || permissions.can_create_folders
            {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "ReadOnly capability cannot include mail-write permissions".to_string(),
                )));
            }
            (true, false)
        }
        MailboxAccessType::SendAs => {
            if permissions.can_read
                || permissions.can_delete
                || permissions.can_move
                || permissions.can_create_folders
                || permissions.can_view_attachments
                || permissions.can_download_attachments
            {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "SendAs capability cannot include read, delete, move, folder, or attachment permissions".to_string(),
                )));
            }
            (false, true)
        }
        // These types require resource-level or non-mail authorization checks
        // that the current remote message entrypoints do not yet implement.
        MailboxAccessType::FolderAccess { .. }
        | MailboxAccessType::ThreadAccess { .. }
        | MailboxAccessType::OutOfOffice
        | MailboxAccessType::OrganizationAdmin
        | MailboxAccessType::Custom(_) => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "This capability access type is not yet enforced end-to-end; refusing to issue it".to_string(),
            )));
        }
    };

    let zome: ZomeName = MESSAGE_ZOME.into();
    let mut functions: HashSet<(ZomeName, FunctionName)> = HashSet::new();
    let mut add = |name: &str| {
        functions.insert((zome.clone(), FunctionName::from(name)));
    };

    if read_allowed && permissions.can_read {
        for name in [
            "get_inbox_v2",
            "get_inbox",
            "get_sent",
            "get_email",
            "get_delivery_receipts",
            "get_drafts",
            "get_folders",
        ] {
            add(name);
        }
        if permissions.can_view_attachments {
            add("get_attachments");
        }
    }

    if write_allowed && permissions.can_send {
        add("send_email_v2");
        add("send_email");
    }
    if write_allowed && permissions.can_delete {
        add("delete_draft");
    }
    if write_allowed && permissions.can_move {
        add("move_to_folder");
    }
    if write_allowed && permissions.can_create_folders {
        add("create_folder");
    }

    if functions.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Capability permissions grant no supported remote mail functions".to_string(),
        )));
    }

    Ok(GrantedFunctions::Listed(functions))
}

fn is_action_permitted(permissions: &MailboxPermissions, action: &AuditAction) -> bool {
    match action {
        AuditAction::ReadEmail => permissions.can_read,
        AuditAction::SendEmail => permissions.can_send,
        AuditAction::DeleteEmail => permissions.can_delete,
        AuditAction::MoveEmail => permissions.can_move,
        AuditAction::CreateFolder => permissions.can_create_folders,
        AuditAction::AccessAttachment => {
            permissions.can_view_attachments && permissions.can_download_attachments
        }
        AuditAction::ModifySettings => permissions.can_modify_settings,
        AuditAction::GrantCapability => permissions.can_delegate,
        AuditAction::ModifyTrust => permissions.can_modify_trust,
        // Unknown/future actions deny by default.
        _ => false,
    }
}

// ==================== REVOKE CAPABILITY ====================

/// Revoke a granted capability
#[hdk_extern]
pub fn revoke_capability(input: (ActionHash, Option<String>)) -> ExternResult<ActionHash> {
    let (cap_hash, reason) = input;
    let my_agent = agent_info()?.agent_initial_pubkey;

    // Get capability
    let record = get(cap_hash.clone(), GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Capability not found".to_string())
    ))?;

    let mut capability: MailboxCapability = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid capability".to_string()
        )))?;

    // Verify ownership
    if capability.grantor != my_agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only grantor can revoke capability".to_string()
        )));
    }

    if capability.revoked {
        return Ok(cap_hash);
    }

    let system_grant_action_hash = capability.system_grant_action_hash.clone().ok_or(
        wasm_error!(WasmErrorInner::Guest(
            "Legacy capability has no bound Holochain grant hash; conductor-level revocation cannot be proven".to_string(),
        )),
    )?;

    // Delete the conductor's private CapGrant first. This is what makes subsequent
    // capability-authenticated remote calls fail Unauthorized; the public app-entry
    // boolean is only an application projection and is not the authorization source.
    delete_cap_grant(system_grant_action_hash)?;

    capability.revoked = true;
    capability.revocation_reason = reason.clone();

    let new_hash = update_entry(
        cap_hash.clone(),
        EntryTypes::MailboxCapability(capability.clone()),
    )?;

    // Signal to grantee
    let signal = CapabilitySignal::CapabilityRevoked {
        capability_id: capability.id,
        grantor: my_agent,
        reason,
    };
    let encoded =
        ExternIO::encode(signal).map_err(|e| wasm_error!(WasmErrorInner::Serialize(e)))?;
    let _ = send_remote_signal(encoded, vec![capability.grantee]);

    // Audit log
    log_capability_action(&cap_hash, AuditAction::RevokeCapability, true, None)?;

    Ok(new_hash)
}

// ==================== VERIFY CAPABILITY ====================

/// Verify if a capability is valid for an action
#[hdk_extern]
pub fn verify_capability(input: (ActionHash, AuditAction)) -> ExternResult<bool> {
    let (cap_hash, action) = input;
    let caller = agent_info()?.agent_initial_pubkey;

    let record = get(cap_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Capability not found".to_string())
    ))?;

    let capability: MailboxCapability = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid capability".to_string()
        )))?;

    // Check if revoked
    if capability.revoked {
        return Ok(false);
    }

    // Check if caller is grantee
    if capability.grantee != caller {
        return Ok(false);
    }

    // Check expiration
    if let Some(expires) = capability.expires_at {
        if expires < sys_time()? {
            return Ok(false);
        }
    }

    Ok(is_action_permitted(&capability.permissions, &action))
}

#[cfg(test)]
mod capability_policy_tests {
    use super::*;

    #[test]
    fn future_actions_fail_closed() {
        let permissions = MailboxPermissions::default();
        assert!(!is_action_permitted(
            &permissions,
            &AuditAction::Custom("future-action".to_string()),
        ));
    }

    #[test]
    fn attachment_access_requires_both_view_and_download() {
        let mut permissions = MailboxPermissions::default();
        permissions.can_view_attachments = true;
        assert!(!is_action_permitted(&permissions, &AuditAction::AccessAttachment));

        permissions.can_download_attachments = true;
        assert!(is_action_permitted(&permissions, &AuditAction::AccessAttachment));
    }

    #[test]
    fn restricted_resource_scopes_are_not_issued_as_broad_grants() {
        let permissions = MailboxPermissions {
            can_read: true,
            ..MailboxPermissions::default()
        };
        assert!(determine_granted_functions(
            &MailboxAccessType::FolderAccess {
                folder_hash: ActionHash::from_raw_36(vec![0; 36]),
            },
            &permissions,
            None,
        ).is_err());
    }
}

// ==================== SHARED MAILBOXES ====================

/// Create a shared mailbox
#[hdk_extern]
pub fn create_shared_mailbox(
    input: (String, String, SharedMailboxSettings),
) -> ExternResult<ActionHash> {
    let (name, email_address, settings) = input;
    let my_agent = agent_info()?.agent_initial_pubkey;
    let now = sys_time()?;

    let id = format!("shared_{}_{}", my_agent, now.as_micros());

    let mailbox = SharedMailbox {
        id,
        name: name.clone(),
        owner: my_agent.clone(),
        email_address,
        members: vec![SharedMailboxMember {
            agent: my_agent.clone(),
            role: SharedMailboxRole::Owner,
            permissions: MailboxPermissions {
                can_read: true,
                can_send: true,
                can_delete: true,
                can_move: true,
                can_create_folders: true,
                can_manage_labels: true,
                can_view_attachments: true,
                can_download_attachments: true,
                can_manage_rules: true,
                can_delegate: true,
                can_modify_settings: true,
                can_view_trust: true,
                can_modify_trust: true,
            },
            added_at: now,
            added_by: my_agent.clone(),
        }],
        created_at: now,
        is_active: true,
        settings,
    };

    let mailbox_hash = create_entry(EntryTypes::SharedMailbox(mailbox))?;

    // Link to owner
    create_link(
        my_agent,
        mailbox_hash.clone(),
        LinkTypes::AgentToOwnedSharedMailboxes,
        LinkTag::new(name.to_string()),
    )?;

    Ok(mailbox_hash)
}

/// Add member to shared mailbox
#[hdk_extern]
pub fn add_shared_mailbox_member(
    input: (
        ActionHash,
        AgentPubKey,
        SharedMailboxRole,
        MailboxPermissions,
    ),
) -> ExternResult<ActionHash> {
    let (mailbox_hash, new_member, role, permissions) = input;
    let my_agent = agent_info()?.agent_initial_pubkey;
    let now = sys_time()?;

    // Get mailbox
    let record = get(mailbox_hash.clone(), GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Shared mailbox not found".to_string())
    ))?;

    let mut mailbox: SharedMailbox = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid mailbox".to_string()
        )))?;

    // Verify caller has permission to add members
    let caller_member = mailbox
        .members
        .iter()
        .find(|m| m.agent == my_agent)
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Not a member of this mailbox".to_string()
        )))?;

    match caller_member.role {
        SharedMailboxRole::Owner | SharedMailboxRole::Admin => {}
        _ => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Insufficient permissions to add members".to_string()
            )));
        }
    }

    // Add new member
    let member = SharedMailboxMember {
        agent: new_member.clone(),
        role: role.clone(),
        permissions,
        added_at: now,
        added_by: my_agent,
    };

    mailbox.members.push(member);

    let new_hash = update_entry(
        mailbox_hash.clone(),
        EntryTypes::SharedMailbox(mailbox.clone()),
    )?;

    // Link to new member
    create_link(
        new_member.clone(),
        mailbox_hash,
        LinkTypes::AgentToMemberSharedMailboxes,
        LinkTag::new(mailbox.name.to_string()),
    )?;

    // Signal to new member
    let signal = CapabilitySignal::AddedToSharedMailbox {
        mailbox_hash: new_hash.clone(),
        mailbox_name: mailbox.name,
        role,
    };
    let encoded =
        ExternIO::encode(signal).map_err(|e| wasm_error!(WasmErrorInner::Serialize(e)))?;
    let _ = send_remote_signal(encoded, vec![new_member]);

    Ok(new_hash)
}

/// Get shared mailboxes user is member of
#[hdk_extern]
pub fn get_my_shared_mailboxes(_: ()) -> ExternResult<Vec<(ActionHash, SharedMailbox)>> {
    let my_agent = agent_info()?.agent_initial_pubkey;

    let mut mailboxes = Vec::new();

    // Get owned mailboxes
    let owned_links = get_links(
        LinkQuery::try_new(my_agent.clone(), LinkTypes::AgentToOwnedSharedMailboxes)?,
        GetStrategy::default(),
    )?;

    // Get member mailboxes
    let member_links = get_links(
        LinkQuery::try_new(my_agent, LinkTypes::AgentToMemberSharedMailboxes)?,
        GetStrategy::default(),
    )?;

    for link in owned_links.into_iter().chain(member_links.into_iter()) {
        let hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid target".to_string())))?;

        if let Some(record) = get(hash.clone(), GetOptions::default())? {
            if let Some(mailbox) = record
                .entry()
                .to_app_option::<SharedMailbox>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                mailboxes.push((hash, mailbox));
            }
        }
    }

    Ok(mailboxes)
}

// ==================== AUDIT LOGGING ====================

fn log_capability_action(
    capability_hash: &ActionHash,
    action: AuditAction,
    success: bool,
    error: Option<String>,
) -> ExternResult<ActionHash> {
    let my_agent = agent_info()?.agent_initial_pubkey;

    let log = CapabilityAuditLog {
        capability_id: capability_hash.to_string(),
        actor: my_agent,
        action,
        resource: Some(capability_hash.clone()),
        timestamp: sys_time()?,
        success,
        error,
        context: None,
    };

    let log_hash = create_entry(EntryTypes::CapabilityAuditLog(log))?;

    create_link(
        capability_hash.clone(),
        log_hash.clone(),
        LinkTypes::CapabilityToAuditLogs,
        LinkTag::new("audit"),
    )?;

    Ok(log_hash)
}

/// Log an action using a capability
#[hdk_extern]
pub fn log_action(
    input: (ActionHash, AuditAction, bool, Option<String>),
) -> ExternResult<ActionHash> {
    let (cap_hash, action, success, error) = input;

    // Verify capability first
    let verified = verify_capability((cap_hash.clone(), action.clone()))?;
    if !verified && success {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Cannot log successful action for invalid capability".to_string()
        )));
    }

    log_capability_action(&cap_hash, action, success, error)
}

/// Get audit logs for a capability
#[hdk_extern]
pub fn get_audit_logs(capability_hash: ActionHash) -> ExternResult<Vec<CapabilityAuditLog>> {
    let links = get_links(
        LinkQuery::try_new(capability_hash, LinkTypes::CapabilityToAuditLogs)?,
        GetStrategy::default(),
    )?;

    let mut logs = Vec::new();

    for link in links {
        let hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid target".to_string())))?;

        if let Some(record) = get(hash, GetOptions::default())? {
            if let Some(log) = record
                .entry()
                .to_app_option::<CapabilityAuditLog>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                logs.push(log);
            }
        }
    }

    // Sort by timestamp
    logs.sort_by(|a, b| b.timestamp.cmp(&a.timestamp));

    Ok(logs)
}

// ==================== QUERY CAPABILITIES ====================

/// Get capabilities I've granted
#[hdk_extern]
pub fn get_granted_capabilities(_: ()) -> ExternResult<Vec<(ActionHash, MailboxCapability)>> {
    let my_agent = agent_info()?.agent_initial_pubkey;

    let links = get_links(
        LinkQuery::try_new(my_agent, LinkTypes::AgentToGrantedCapabilities)?,
        GetStrategy::default(),
    )?;

    let mut capabilities = Vec::new();

    for link in links {
        let hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid target".to_string())))?;

        if let Some(record) = get(hash.clone(), GetOptions::default())? {
            if let Some(cap) = record
                .entry()
                .to_app_option::<MailboxCapability>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                capabilities.push((hash, cap));
            }
        }
    }

    Ok(capabilities)
}

/// Get capabilities I've received
#[hdk_extern]
pub fn get_received_capabilities(_: ()) -> ExternResult<Vec<(ActionHash, MailboxCapability)>> {
    let my_agent = agent_info()?.agent_initial_pubkey;

    let links = get_links(
        LinkQuery::try_new(my_agent, LinkTypes::AgentToReceivedCapabilities)?,
        GetStrategy::default(),
    )?;

    let mut capabilities = Vec::new();

    for link in links {
        let hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid target".to_string())))?;

        if let Some(record) = get(hash.clone(), GetOptions::default())? {
            if let Some(cap) = record
                .entry()
                .to_app_option::<MailboxCapability>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                // Only return non-revoked, non-expired
                if !cap.revoked {
                    if let Some(expires) = cap.expires_at {
                        if expires > sys_time()? {
                            capabilities.push((hash, cap));
                        }
                    } else {
                        capabilities.push((hash, cap));
                    }
                }
            }
        }
    }

    Ok(capabilities)
}

// ==================== CAPABILITY SECRET HANDOFF ====================

/// Deliver a committed capability to its assigned recipient. This must be a
/// separate invocation from grant_capability so both the public record and private
/// system grant have committed before the receiver validates and claims them.
#[hdk_extern]
pub fn deliver_capability_grant(capability_hash: ActionHash) -> ExternResult<()> {
    let local_agent = agent_info()?.agent_initial_pubkey;
    let record = get(capability_hash.clone(), GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Capability record not found".to_string())
    ))?;
    let capability: MailboxCapability = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Unexpected entry type for capability delivery".to_string()
        )))?;

    if capability.grantor != local_agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the grantor can deliver this capability".to_string()
        )));
    }
    if capability.revoked {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Cannot deliver a revoked capability".to_string()
        )));
    }

    let grant_hash = capability.system_grant_action_hash.clone().ok_or(
        wasm_error!(WasmErrorInner::Guest(
            "Capability has no bound Holochain grant hash".to_string()
        )),
    )?;
    let grant_record = get(grant_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Private system grant not found on the grantor chain".to_string())
    ))?;
    let grant = match grant_record.entry().as_option() {
        Some(Entry::CapGrant(grant)) => grant.clone(),
        _ => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Bound action is not a Holochain capability grant".to_string()
            )));
        }
    };

    if grant.tag != capability.id {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "System grant tag does not match the application capability".to_string()
        )));
    }
    let secret = match grant.access {
        CapAccess::Assigned { secret, assignees }
            if assignees.contains(&capability.grantee) => secret,
        _ => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "System grant is not assigned to the recorded grantee".to_string()
            )));
        }
    };
    if Sha256::digest(secret.as_ref()).to_vec() != capability.secret_hash {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "System grant secret fingerprint does not match the application record".to_string()
        )));
    }

    let delivery = CapabilityGrantDelivery {
        capability_hash: capability_hash.clone(),
        capability_id: capability.id.clone(),
        grantor: local_agent,
        access_type: capability.access_type.clone(),
        secret,
    };

    let response = call_remote(
        capability.grantee.clone(),
        zome_info()?.name,
        FunctionName::from("receive_capability_grant"),
        None,
        delivery,
    )?;
    match response {
        ZomeCallResponse::Ok(_) => {}
        ZomeCallResponse::Unauthorized(_, _, _, _, _) => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Recipient rejected capability delivery as Unauthorized".to_string()
            )));
        }
        _ => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Capability delivery failed without an acknowledgement".to_string()
            )));
        }
    }

    // The secret never appears in a UI signal.
    emit_signal(CapabilitySignal::CapabilityGranted {
        capability_hash,
        grantor: local_agent,
        access_type: capability.access_type,
    })?;
    Ok(())
}

/// Receive a capability from the authenticated grantor and persist it as a private
/// local CapClaim. The grantor must use the separate post-commit delivery call.
#[hdk_extern]
pub fn receive_capability_grant(delivery: CapabilityGrantDelivery) -> ExternResult<()> {
    let caller = call_info()?.provenance;
    let local_agent = agent_info()?.agent_initial_pubkey;

    if caller != delivery.grantor {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Capability delivery source does not match grantor".to_string()
        )));
    }

    let record = get(delivery.capability_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Capability metadata is not yet available; retry delivery after DHT publication".to_string()
        )))?;
    let capability: MailboxCapability = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Capability record has an unexpected entry type".to_string()
        )))?;

    if capability.grantor != caller
        || capability.grantee != local_agent
        || capability.id != delivery.capability_id
        || capability.revoked
        || capability.secret_hash != Sha256::digest(delivery.secret.as_ref()).to_vec()
        || capability.access_type != delivery.access_type
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Capability delivery does not match the authenticated public grant record".to_string()
        )));
    }

    if let Some(existing) = find_cap_claim(&delivery.grantor, &delivery.capability_id)? {
        if existing.secret.as_ref() != delivery.secret.as_ref() {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Conflicting claim already exists for this grantor and capability ID".to_string()
            )));
        }
    } else {
        create_cap_claim(CapClaimEntry {
            tag: delivery.capability_id,
            grantor: delivery.grantor.clone(),
            secret: delivery.secret,
        })?;
    }

    // This local-only signal contains no capability secret.
    emit_signal(CapabilitySignal::CapabilityGranted {
        capability_hash: delivery.capability_hash,
        grantor: delivery.grantor,
        access_type: delivery.access_type,
    })?;
    Ok(())
}

// ==================== REMOTE CAPABILITY QUALIFICATION ====================

/// Result of a real conductor-authorized remote call. This deliberately does not
/// infer authorization from the public MailboxCapability.revoked projection.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CapabilityProbeResult {
    Authorized,
    Unauthorized,
    ClaimMissing,
}

/// Find the private source-chain claim corresponding to one grantor + capability tag.
fn find_cap_claim(grantor: &AgentPubKey, tag: &str) -> ExternResult<Option<CapClaim>> {
    let filter = ChainQueryFilter::new()
        .action_type(ActionType::Create)
        .entry_type(EntryType::CapClaim)
        .include_entries(true);

    let matching_claim = query(filter)?.into_iter().find_map(|record| {
        let entry = record.entry().as_option()?;
        match entry {
            Entry::CapClaim(claim) if claim.grantor == *grantor && claim.tag == tag => {
                Some(claim.clone())
            }
            _ => None,
        }
    });

    Ok(matching_claim)
}

/// Probe the actual Holochain grant by making a harmless, data-discarding remote
/// call to get_inbox_v2. The app-entry revoked flag is intentionally not an early
/// return here: this diagnostic must distinguish an application projection from
/// actual conductor authorization. It returns no mailbox contents.
#[hdk_extern]
pub fn probe_remote_capability(capability_hash: ActionHash) -> ExternResult<CapabilityProbeResult> {
    let local_agent = agent_info()?.agent_initial_pubkey;
    let record = get(capability_hash.clone(), GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Capability record not found".to_string())
    ))?;
    let capability: MailboxCapability = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Unexpected entry type for capability probe".to_string()
        )))?;

    if capability.grantee != local_agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the assigned grantee can probe this capability".to_string()
        )));
    }

    let Some(claim) = find_cap_claim(&capability.grantor, &capability.id)? else {
        return Ok(CapabilityProbeResult::ClaimMissing);
    };

    let response = call_remote(
        capability.grantor,
        ZomeName::from("mail_messages"),
        FunctionName::from("get_inbox_v2"),
        Some(claim.secret),
        (),
    )?;

    match response {
        ZomeCallResponse::Ok(_) => Ok(CapabilityProbeResult::Authorized),
        ZomeCallResponse::Unauthorized(_, _, _, _, _) => {
            Ok(CapabilityProbeResult::Unauthorized)
        }
        _ => Err(wasm_error!(WasmErrorInner::Guest(
            "Remote capability probe failed for a reason other than authorization".to_string()
        ))),
    }
}

// ==================== SIGNAL HANDLING ====================

#[hdk_extern]
pub fn recv_remote_signal(signal: ExternIO) -> ExternResult<()> {
    // Grant delivery is a separate wire type so the secret can be consumed into a
    // private CapClaim without ever being forwarded to local UI listeners.
    let cap_signal = match signal.decode::<CapabilityGrantDelivery>() {
        Ok(delivery) => {
            let caller = call_info()?.provenance;
            let local_agent = agent_info()?.agent_initial_pubkey;
            if caller != delivery.grantor {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Capability grant delivery source does not match grantor".to_string(),
                )));
            }

            let record = get(delivery.capability_hash.clone(), GetOptions::default())?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Capability grant record unavailable to recipient".to_string(),
                )))?;
            let capability: MailboxCapability = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Capability grant record has an unexpected entry type".to_string(),
                )))?;

            if capability.grantor != caller
                || capability.grantee != local_agent
                || capability.id != delivery.capability_id
                || capability.secret_hash != Sha256::digest(delivery.secret.as_ref()).to_vec()
            {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Capability secret does not match the authorized grant record".to_string(),
                )));
            }

            create_cap_claim(CapClaimEntry {
                tag: delivery.capability_id,
                grantor: delivery.grantor.clone(),
                secret: delivery.secret,
            })?;

            CapabilitySignal::CapabilityGranted {
                capability_hash: delivery.capability_hash,
                grantor: delivery.grantor,
                access_type: delivery.access_type,
            }
        }
        Err(_) => {
            let cap_signal: CapabilitySignal = signal.decode().map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode capability signal: {}",
                    e
                )))
            })?;

            if let CapabilitySignal::CapabilityRevoked { grantor, .. } = &cap_signal {
                if call_info()?.provenance != *grantor {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "Capability revocation source does not match grantor".to_string(),
                    )));
                }
            }
            cap_signal
        }
    };

    emit_signal(cap_signal)?;
    Ok(())
}

// ==================== INIT ====================

#[hdk_extern]
pub fn init(_: ()) -> ExternResult<InitCallbackResult> {
    // Grant capability for receiving signals
    let zome = zome_info()?.name;
    let functions = GrantedFunctions::Listed(HashSet::from([
        (zome.clone(), FunctionName::from("recv_remote_signal")),
        (zome, FunctionName::from("receive_capability_grant")),
    ]));

    create_cap_grant(CapGrantEntry {
        tag: "recv_cap_signals".to_string(),
        access: CapAccess::Unrestricted,
        functions,
    })?;

    Ok(InitCallbackResult::Pass)
}
