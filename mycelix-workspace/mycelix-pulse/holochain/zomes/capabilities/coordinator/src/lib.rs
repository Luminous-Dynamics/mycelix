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
pub struct CapabilityGrantDelivery {
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
    let secret = generate_cap_secret()?;
    let secret_hash = Sha256::digest(secret.as_ref()).to_vec();
    // The host-generated secret adds collision resistance even if two grant
    // transactions share the same timestamp resolution.
    let entropy_suffix = secret_hash
        .iter()
        .take(8)
        .map(|byte| format!("{byte:02x}"))
        .collect::<String>();
    let id = format!("cap_{}_{}_{}", my_agent, now.as_micros(), entropy_suffix);
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
        // This empty-response endpoint lets the grantee test actual conductor
        // enforcement without transmitting every encrypted inbox item.
        for name in [
            "capability_probe_v1",
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
    let (requested_hash, reason) = input;
    let my_agent = agent_info()?.agent_initial_pubkey;
    let (cap_hash, mut capability) = resolve_latest_capability(requested_hash)?;

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

/// Check the currently visible application capability projection for an audit action.
///
/// This is not an access-control boundary or proof of globally current DHT state:
/// a remote agent may not yet have received a later revocation. Protected remote
/// zome calls must rely on Holochain's actual CapGrant enforcement. The
/// probe_remote_capability endpoint separately exercises the assigned inbox grant
/// and reports the actual conductor response for that specific remote call.
#[hdk_extern]
pub fn verify_capability(input: (ActionHash, AuditAction)) -> ExternResult<bool> {
    let (cap_hash, action) = input;
    let caller = agent_info()?.agent_initial_pubkey;
    let (_latest_hash, capability) = resolve_latest_capability(cap_hash)?;

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

    /// Compare the complete remote-call allowlist, not a handful of forbidden
    /// functions. This makes adding a new exported function fail closed until
    /// capability policy explicitly qualifies it.
    fn assert_exact_scope(grant: GrantedFunctions, expected_names: &[&str]) {
        let expected: HashSet<(ZomeName, FunctionName)> = expected_names
            .iter()
            .map(|name| {
                (
                    ZomeName::from("mail_messages"),
                    FunctionName::from(*name),
                )
            })
            .collect();

        match grant {
            GrantedFunctions::Listed(functions) => assert_eq!(
                functions, expected,
                "capability must expose exactly the explicitly qualified function set"
            ),
            GrantedFunctions::All => {
                panic!("capability must never receive an unrestricted function grant");
            }
        }
    }

    #[test]
    fn read_only_grant_has_exact_read_scope() {
        let permissions = MailboxPermissions {
            can_read: true,
            ..MailboxPermissions::default()
        };
        let grant = determine_granted_functions(
            &MailboxAccessType::ReadOnly,
            &permissions,
            None,
        )
        .expect("read-only grant should be supported");

        assert_exact_scope(
            grant,
            &[
                "capability_probe_v1",
                "get_inbox_v2",
                "get_inbox",
                "get_sent",
                "get_email",
                "get_delivery_receipts",
                "get_drafts",
                "get_folders",
            ],
        );
    }

    #[test]
    fn send_as_grant_has_exact_send_scope() {
        let permissions = MailboxPermissions {
            can_send: true,
            ..MailboxPermissions::default()
        };
        let grant = determine_granted_functions(
            &MailboxAccessType::SendAs,
            &permissions,
            None,
        )
        .expect("send-only grant should be supported");

        assert_exact_scope(grant, &["send_email_v2", "send_email"]);
    }

    #[test]
    fn attachment_permission_adds_only_the_attachment_read_endpoint() {
        let permissions = MailboxPermissions {
            can_read: true,
            can_view_attachments: true,
            can_download_attachments: true,
            ..MailboxPermissions::default()
        };
        let grant = determine_granted_functions(
            &MailboxAccessType::ReadOnly,
            &permissions,
            None,
        )
        .expect("attachment read grant should be supported");

        assert_exact_scope(
            grant,
            &[
                "capability_probe_v1",
                "get_inbox_v2",
                "get_inbox",
                "get_sent",
                "get_email",
                "get_delivery_receipts",
                "get_drafts",
                "get_folders",
                "get_attachments",
            ],
        );
    }

    #[test]
    fn every_unimplemented_permission_fails_closed() {
        let cases: Vec<(&str, MailboxPermissions)> = vec![
            (
                "can_manage_labels",
                MailboxPermissions {
                    can_manage_labels: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "can_manage_rules",
                MailboxPermissions {
                    can_manage_rules: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "can_delegate",
                MailboxPermissions {
                    can_delegate: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "can_modify_settings",
                MailboxPermissions {
                    can_modify_settings: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "can_view_trust",
                MailboxPermissions {
                    can_view_trust: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "can_modify_trust",
                MailboxPermissions {
                    can_modify_trust: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "can_delete",
                MailboxPermissions {
                    can_delete: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "read-only cannot send",
                MailboxPermissions {
                    can_read: true,
                    can_send: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "read-only cannot move",
                MailboxPermissions {
                    can_read: true,
                    can_move: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "read-only cannot create folders",
                MailboxPermissions {
                    can_read: true,
                    can_create_folders: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "attachments require read access",
                MailboxPermissions {
                    can_view_attachments: true,
                    can_download_attachments: true,
                    ..MailboxPermissions::default()
                },
            ),
            (
                "attachment view/download permissions must agree",
                MailboxPermissions {
                    can_read: true,
                    can_view_attachments: true,
                    ..MailboxPermissions::default()
                },
            ),
        ];

        for (case, permissions) in cases {
            assert!(
                determine_granted_functions(
                    &MailboxAccessType::ReadOnly,
                    &permissions,
                    None,
                )
                .is_err(),
                "unsupported permission profile must fail closed: {case}"
            );
        }
    }

    #[test]
    fn every_unenforced_resource_restriction_fails_closed() {
        let base = AccessRestrictions {
            folder_whitelist: None,
            folder_blacklist: None,
            sender_whitelist: None,
            max_emails: None,
            date_from: None,
            date_to: None,
            network_restrictions: None,
            require_2fa: false,
            audit_required: false,
        };
        let folder = ActionHash::from_raw_36(vec![0; 36]);
        let first_date = Timestamp::from_micros(1);
        let last_date = Timestamp::from_micros(2);
        let cases: Vec<(&str, AccessRestrictions)> = vec![
            (
                "folder_whitelist",
                AccessRestrictions {
                    folder_whitelist: Some(vec![folder.clone()]),
                    ..base.clone()
                },
            ),
            (
                "folder_blacklist",
                AccessRestrictions {
                    folder_blacklist: Some(vec![folder]),
                    ..base.clone()
                },
            ),
            (
                "sender_whitelist",
                AccessRestrictions {
                    sender_whitelist: Some(vec!["sender@example.test".to_string()]),
                    ..base.clone()
                },
            ),
            (
                "max_emails",
                AccessRestrictions {
                    max_emails: Some(1),
                    ..base.clone()
                },
            ),
            (
                "date_from",
                AccessRestrictions {
                    date_from: Some(first_date),
                    ..base.clone()
                },
            ),
            (
                "date_to",
                AccessRestrictions {
                    date_to: Some(last_date),
                    ..base.clone()
                },
            ),
            (
                "network_restrictions",
                AccessRestrictions {
                    network_restrictions: Some(vec!["10.0.0.0/8".to_string()]),
                    ..base.clone()
                },
            ),
            (
                "require_2fa",
                AccessRestrictions {
                    require_2fa: true,
                    ..base.clone()
                },
            ),
            (
                "audit_required",
                AccessRestrictions {
                    audit_required: true,
                    ..base
                },
            ),
        ];
        let permissions = MailboxPermissions {
            can_read: true,
            ..MailboxPermissions::default()
        };

        for (case, restrictions) in cases {
            assert!(
                determine_granted_functions(
                    &MailboxAccessType::ReadOnly,
                    &permissions,
                    Some(&restrictions),
                )
                .is_err(),
                "unenforced resource restriction must fail closed: {case}"
            );
        }
    }

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
    let mut seen = HashSet::new();

    for link in links {
        let hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid target".to_string())))?;
        let (latest_hash, capability) = resolve_latest_capability(hash)?;
        if seen.insert(capability.id.clone()) {
            capabilities.push((latest_hash, capability));
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

    let now = sys_time()?;
    let mut capabilities = Vec::new();
    let mut seen = HashSet::new();

    for link in links {
        let hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid target".to_string())))?;
        let (latest_hash, capability) = resolve_latest_capability(hash)?;
        if !seen.insert(capability.id.clone()) || capability.revoked {
            continue;
        }
        if capability.expires_at.is_some_and(|expires| expires <= now) {
            continue;
        }
        capabilities.push((latest_hash, capability));
    }

    Ok(capabilities)
}

// ==================== CAPABILITY SECRET HANDOFF ====================

/// Deliver a committed capability to its assigned recipient. This must be a
/// separate invocation from grant_capability so both the public record and private
/// system grant have committed before the receiver validates and claims them.
#[hdk_extern]
pub fn deliver_capability_grant(requested_hash: ActionHash) -> ExternResult<()> {
    let local_agent = agent_info()?.agent_initial_pubkey;
    let (capability_hash, capability) = resolve_latest_capability(requested_hash)?;

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

    if let Some(existing) = find_cap_claim(
        &delivery.grantor,
        &delivery.capability_id,
        &capability.secret_hash,
    )? {
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
    /// The empty-response capability probe returned successfully under the conductor's grant.
    Authorized,
    /// The capability probe call was rejected by conductor-level authorization.
    /// This alone does not distinguish revocation from another grant/secret mismatch.
    Unauthorized,
    /// No matching private claim has the secret fingerprint bound to the public grant.
    ClaimMissing,
    /// The empty-response probe endpoint was not included in the requested capability permissions.
    FunctionNotGranted,
}

/// Resolve the latest capability update visible to this cell by walking update actions.
/// A get(original_hash) returns the original record; it does not automatically follow
/// updates. This is an application projection, NOT proof of globally current DHT state:
/// conductor CapGrant enforcement remains authoritative for protected remote calls.
/// If an update is referenced but its evidence is unavailable/incomplete, return an
/// explicit error rather than silently treating the older projection as current.
fn resolve_latest_capability(
    capability_hash: ActionHash,
) -> ExternResult<(ActionHash, MailboxCapability)> {
    let record = get(capability_hash.clone(), GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Capability record not found".to_string())
    ))?;
    let capability: MailboxCapability = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Unexpected entry type for capability record".to_string()
        )))?;

    // Capability updates are grantor-authored and validated as a single
    // active -> revoked transition. Compare source-chain action sequence, not
    // wall-clock timestamps or hash lexical order (both can tie or misorder).
    let mut latest = (
        record.action().action_seq(),
        capability_hash.clone(),
        capability,
    );
    let mut frontier = vec![capability_hash];
    let mut visited = HashSet::new();

    while let Some(parent_hash) = frontier.pop() {
        let Some(details) = get_details(parent_hash, GetOptions::default())? else {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Capability state is unknown: update details are unavailable; refusing a potentially stale projection".to_string(),
            )));
        };
        let Details::Record(details) = details else {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Capability state is unknown: update details did not resolve to record details".to_string(),
            )));
        };

        for update in details.updates {
            let update_hash = update.hashed.hash.clone();
            if !visited.insert(update_hash.clone()) {
                continue;
            }
            let Some(update_record) = get(update_hash.clone(), GetOptions::default())? else {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Capability state is unknown: a referenced update record is unavailable".to_string(),
                )));
            };
            let Some(updated_capability) = update_record
                .entry()
                .to_app_option::<MailboxCapability>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            else {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Capability state is unknown: a referenced update does not contain a readable MailboxCapability entry".to_string(),
                )));
            };

            let action_seq = update_record.action().action_seq();
            if action_seq > latest.0 {
                latest = (action_seq, update_hash.clone(), updated_capability);
            }
            frontier.push(update_hash);
        }
    }

    Ok((latest.1, latest.2))
}

/// Find a private CapClaim only when its grantor, tag, and secret fingerprint
/// match the public capability record. A tag-only match could select an unrelated claim.
fn find_cap_claim(
    grantor: &AgentPubKey,
    tag: &str,
    expected_secret_hash: &[u8],
) -> ExternResult<Option<CapClaim>> {
    let filter = ChainQueryFilter::new()
        .action_type(ActionType::Create)
        .entry_type(EntryType::CapClaim)
        .include_entries(true);

    let matching_claim = query(filter)?.into_iter().find_map(|record| {
        let entry = record.entry().as_option()?;
        match entry {
            Entry::CapClaim(claim)
                if claim.grantor == *grantor
                    && claim.tag == tag
                    && Sha256::digest(claim.secret.as_ref())
                        .iter()
                        .eq(expected_secret_hash.iter()) =>
            {
                Some(claim.clone())
            }
            _ => None,
        }
    });

    Ok(matching_claim)
}

/// Probe the actual Holochain grant with an empty-response remote call.
/// The app-entry revoked flag is intentionally not an early return here: this
/// diagnostic distinguishes the application projection from conductor authorization
/// without transmitting mailbox contents.
#[hdk_extern]
pub fn probe_remote_capability(capability_hash: ActionHash) -> ExternResult<CapabilityProbeResult> {
    let local_agent = agent_info()?.agent_initial_pubkey;

    // Deliberately load the original capability identity, not the latest
    // application projection. This probe exists to test actual conductor
    // enforcement even while the caller's DHT view of later updates is stale
    // or incomplete. Mutable status fields are not used to authorize the call.
    let record = get(capability_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Capability record not found".to_string())
    ))?;
    let capability: MailboxCapability = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Unexpected entry type for capability record".to_string()
        )))?;

    if capability.grantee != local_agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the assigned grantee can probe this capability".to_string()
        )));
    }

    // This diagnostic probes the empty-response capability_probe_v1 endpoint.
    // A send-only grant must not report Unauthorized and thereby masquerade as
    // a revoked read grant; FunctionNotGranted is an application-scope result.
    if !capability.permissions.can_read
        || !matches!(
            capability.access_type,
            MailboxAccessType::FullAccess | MailboxAccessType::ReadOnly
        )
    {
        return Ok(CapabilityProbeResult::FunctionNotGranted);
    }

    let Some(claim) = find_cap_claim(
        &capability.grantor,
        &capability.id,
        &capability.secret_hash,
    )? else {
        return Ok(CapabilityProbeResult::ClaimMissing);
    };

    let response = call_remote(
        capability.grantor,
        ZomeName::from("mail_messages"),
        FunctionName::from("capability_probe_v1"),
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

/// Diagnostic for qualification tests: exercise the actual inbox-read function
/// under the assigned grant, but return only its authorization outcome to the
/// local caller. The remote inbox payload still reaches this zome's WASM memory;
/// do not use this instead of the normal no-data probe in routine UI polling.
#[hdk_extern]
pub fn probe_remote_inbox_read(capability_hash: ActionHash) -> ExternResult<CapabilityProbeResult> {
    let local_agent = agent_info()?.agent_initial_pubkey;
    let record = get(capability_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Capability record not found".to_string())
    ))?;
    let capability: MailboxCapability = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Unexpected entry type for capability record".to_string()
        )))?;

    if capability.grantee != local_agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the assigned grantee can probe this capability".to_string()
        )));
    }
    if !capability.permissions.can_read
        || !matches!(
            capability.access_type,
            MailboxAccessType::FullAccess | MailboxAccessType::ReadOnly
        )
    {
        return Ok(CapabilityProbeResult::FunctionNotGranted);
    }

    let Some(claim) = find_cap_claim(
        &capability.grantor,
        &capability.id,
        &capability.secret_hash,
    )? else {
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
            "Remote inbox-read qualification call failed for a reason other than authorization"
                .to_string()
        ))),
    }
}

// ==================== SIGNAL HANDLING ====================

#[hdk_extern]
pub fn recv_remote_signal(signal: ExternIO) -> ExternResult<()> {
    // Secrets are never delivered through best-effort remote signals. The only
    // accepted grant-delivery path is receive_capability_grant via call_remote,
    // which returns an acknowledgement after the private CapClaim is stored.
    let cap_signal: CapabilitySignal = signal.decode().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Failed to decode capability signal: {}",
            e
        )))
    })?;

    match &cap_signal {
        CapabilitySignal::CapabilityRevoked { grantor, .. } => {
            if call_info()?.provenance != *grantor {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Capability revocation source does not match grantor".to_string(),
                )));
            }
        }
        CapabilitySignal::CapabilityGranted { .. } => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "CapabilityGranted must arrive through the acknowledged private-claim handoff".to_string(),
            )));
        }
        _ => {}
    }

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
