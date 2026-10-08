// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Machines Integrity Zome
//!
//! Entry types and validation for the machine registry.

use hdi::prelude::*;
use manufacturing_common::{MachineStatus, MachineType};

/// Maximum controller lease duration for the initial deterministic authority model.
pub const MAX_MACHINE_CONTROLLER_LEASE_MICROS: i64 = 86_400_000_000;
/// Domain-separated payload signed by the machine registrant for a controller lease.
pub const MACHINE_CONTROLLER_LEASE_SCHEMA_ID_V1: &str =
    "mycelix-manufacturing-machine-controller-lease-v1";
pub const MACHINE_CONTROLLER_LEASE_SCHEMA_ID_V2: &str =
    "mycelix-manufacturing-machine-controller-lease-v2";
pub const MACHINE_CONTROLLER_TRANSITION_APPROVAL_SCHEMA_ID: &str =
    "mycelix-manufacturing-machine-transition-approval-v1";
pub const MAX_MACHINE_TRANSITION_APPROVAL_MICROS: i64 = 300_000_000;

pub const MACHINE_TIME_AUTHORITY_PROFILE_SCHEMA_ID: &str =
    "mycelix-manufacturing-machine-time-authority-profile-v1";
pub const MACHINE_TEMPORAL_ATTESTATION_SCHEMA_ID: &str =
    "mycelix-manufacturing-machine-temporal-attestation-v1";

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MachineTemporalEvidenceResolution {
    NoEvidence,
    Unique(Timestamp),
    Conflicting(Vec<Timestamp>),
    InvalidEvidence,
}

fn resolve_temporal_times(mut times: Vec<Timestamp>) -> MachineTemporalEvidenceResolution {
    times.sort_by_key(|time| time.as_micros());
    times.dedup();
    match times.as_slice() {
        [] => MachineTemporalEvidenceResolution::NoEvidence,
        [time] => MachineTemporalEvidenceResolution::Unique(*time),
        _ => MachineTemporalEvidenceResolution::Conflicting(times),
    }
}
pub enum MachineTemporalEvidenceKind {
    TransitionApproval,
    MachineActionExistence,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineTimeAuthorityProfilePayload {
    pub schema_id: String,
    pub machine_hash: ActionHash,
    pub authority_agent: AgentPubKey,
    pub profile_id: String,
    pub source_profile: String,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineTemporalAttestationPayload {
    pub schema_id: String,
    pub machine_hash: ActionHash,
    pub profile_hash: ActionHash,
    pub subject_hash: ActionHash,
    pub evidence_kind: MachineTemporalEvidenceKind,
    pub attested_at: Timestamp,
    pub source_reference: String,
}

fn default_lease_schema_version() -> u8 {
    1
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineControllerLeasePayloadV1 {
    pub schema_id: String,
    pub machine_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineControllerLeasePayloadV2 {
    pub schema_id: String,
    pub machine_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub requires_transition_approval: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineControllerTransitionApprovalPayload {
    pub schema_id: String,
    pub machine_hash: ActionHash,
    pub authority_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub predecessor_action: ActionHash,
    pub new_status: MachineStatus,
    pub work_order_hash: Option<ActionHash>,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
}
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineEntry {
    pub name: String,
    pub machine_type: MachineType,
    pub capabilities: Vec<String>,
    pub location: String,
    pub max_throughput_per_hour: u32,
    pub status: MachineStatus,
    pub current_work_order: Option<ActionHash>,
    #[serde(default)]
    pub last_status_authority_hash: Option<ActionHash>,
    #[serde(default)]
    pub last_status_transition_approval_hash: Option<ActionHash>,
    pub registered_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineTimeAuthorityProfileEntry {
    pub machine_hash: ActionHash,
    pub authority_agent: AgentPubKey,
    pub profile_id: String,
    pub source_profile: String,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub registrant_signature: Signature,
}

impl MachineTimeAuthorityProfileEntry {
    pub fn signed_payload(&self) -> MachineTimeAuthorityProfilePayload {
        MachineTimeAuthorityProfilePayload {
            schema_id: MACHINE_TIME_AUTHORITY_PROFILE_SCHEMA_ID.to_string(),
            machine_hash: self.machine_hash.clone(),
            authority_agent: self.authority_agent.clone(),
            profile_id: self.profile_id.clone(),
            source_profile: self.source_profile.clone(),
            valid_from: self.valid_from,
            valid_until: self.valid_until,
        }
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineTemporalAttestationEntry {
    pub machine_hash: ActionHash,
    pub profile_hash: ActionHash,
    pub subject_hash: ActionHash,
    pub evidence_kind: MachineTemporalEvidenceKind,
    pub attested_at: Timestamp,
    pub source_reference: String,
    pub authority_signature: Signature,
}

impl MachineTemporalAttestationEntry {
    pub fn signed_payload(&self) -> MachineTemporalAttestationPayload {
        MachineTemporalAttestationPayload {
            schema_id: MACHINE_TEMPORAL_ATTESTATION_SCHEMA_ID.to_string(),
            machine_hash: self.machine_hash.clone(),
            profile_hash: self.profile_hash.clone(),
            subject_hash: self.subject_hash.clone(),
            evidence_kind: self.evidence_kind.clone(),
            attested_at: self.attested_at,
            source_reference: self.source_reference.clone(),
        }
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineControllerAuthorityEntry {
    pub machine_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    #[serde(default = "default_lease_schema_version")]
    pub lease_schema_version: u8,
    #[serde(default)]
    pub requires_transition_approval: bool,
    /// Cryptographic proof that the machine registrant authored the exact lease fields.
    /// None is migration compatibility only; unsigned legacy authorities cannot be
    /// used for new control-field updates.
    #[serde(default)]
    pub issuer_signature: Option<Signature>,
}

impl MachineControllerAuthorityEntry {
    pub fn signed_payload_v1(&self) -> MachineControllerLeasePayloadV1 {
        MachineControllerLeasePayloadV1 {
            schema_id: MACHINE_CONTROLLER_LEASE_SCHEMA_ID_V1.to_string(),
            machine_hash: self.machine_hash.clone(),
            controller_agent: self.controller_agent.clone(),
            valid_from: self.valid_from,
            valid_until: self.valid_until,
        }
    }

    pub fn signed_payload_v2(&self) -> MachineControllerLeasePayloadV2 {
        MachineControllerLeasePayloadV2 {
            schema_id: MACHINE_CONTROLLER_LEASE_SCHEMA_ID_V2.to_string(),
            machine_hash: self.machine_hash.clone(),
            controller_agent: self.controller_agent.clone(),
            valid_from: self.valid_from,
            valid_until: self.valid_until,
            requires_transition_approval: true,
        }
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineControllerTransitionApprovalEntry {
    pub machine_hash: ActionHash,
    pub authority_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub predecessor_action: ActionHash,
    pub new_status: MachineStatus,
    pub work_order_hash: Option<ActionHash>,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub issuer_signature: Signature,
}

impl MachineControllerTransitionApprovalEntry {
    pub fn signed_payload(&self) -> MachineControllerTransitionApprovalPayload {
        MachineControllerTransitionApprovalPayload {
            schema_id: MACHINE_CONTROLLER_TRANSITION_APPROVAL_SCHEMA_ID.to_string(),
            machine_hash: self.machine_hash.clone(),
            authority_hash: self.authority_hash.clone(),
            controller_agent: self.controller_agent.clone(),
            predecessor_action: self.predecessor_action.clone(),
            new_status: self.new_status.clone(),
            work_order_hash: self.work_order_hash.clone(),
            valid_from: self.valid_from,
            valid_until: self.valid_until,
        }
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineStatusLog {
    pub machine_hash: ActionHash,
    pub machine_update_hash: ActionHash,
    pub authority_hash: ActionHash,
    pub previous_status: MachineStatus,
    pub new_status: MachineStatus,
    pub work_order_hash: Option<ActionHash>,
    #[serde(default)]
    pub transition_approval_hash: Option<ActionHash>,
    pub changed_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Machine(MachineEntry),
    StatusLog(MachineStatusLog),
    MachineControllerAuthority(MachineControllerAuthorityEntry),
    MachineControllerTransitionApproval(MachineControllerTransitionApprovalEntry),
    MachineTimeAuthorityProfile(MachineTimeAuthorityProfileEntry),
    MachineTemporalAttestation(MachineTemporalAttestationEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllMachines,
    TypeToMachines,
    MachineToStatusLog,
    MachineUpdateToStatusLog,
    MachineToAuthorities,
    AllMachineControllerAuthorities,
    MachineToTransitionApprovals,
    AllMachineTransitionApprovals,
    MachineToTimeAuthorityProfiles,
    AllMachineTimeAuthorityProfiles,
    MachineToTemporalAttestations,
    AllMachineTemporalAttestations,
    SubjectToTemporalAttestations,
    LocationToMachines,
}

/// **P0 author-binding pass, 2026-07-09**: no identity field exists on
/// either entry (case a -- shared machine registry, not per-agent-owned).
/// What IS fixed: updates were previously routed through the wide-open
/// catch-all `_ => Valid` -- `update_machine_status`'s `update_entry`
/// call was accepted with zero validation, so a modified coordinator
/// could silently rewrite ANY field (name, machine_type, capabilities,
/// location, max_throughput_per_hour), not just the intended
/// status/current_work_order, and could skip the state-machine
/// transition check entirely. Fixed via must_get content-restriction
/// plus re-running `can_transition_to`.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action }) => {
            match app_entry {
                EntryTypes::MachineControllerAuthority(authority) => {
                    validate_create_authority(action, authority)
                }
                EntryTypes::StatusLog(log) => validate_create_status_log(action, log),
                EntryTypes::MachineControllerTransitionApproval(approval) => {
                    validate_create_transition_approval(action, approval)
                }
                EntryTypes::MachineTimeAuthorityProfile(profile) => {
                    validate_create_time_authority_profile(action, profile)
                }
                EntryTypes::MachineTemporalAttestation(attestation) => {
                    validate_create_temporal_attestation(action, attestation)
                }
                other => validate_create_entry(other),
            }
        }
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry,
            original_action_hash,
            action,
            ..
        }) => validate_update_entry(original_action_hash, action, app_entry),
        FlatOp::RegisterUpdate(OpUpdate::Entry {
            app_entry, action, ..
        }) => validate_update_entry(action.original_action_address, action, app_entry),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Machine registry records are immutable".into(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_time_authority_profile(
    action: TypedAction<CreateData>,
    profile: MachineTimeAuthorityProfileEntry,
) -> ExternResult<ValidateCallbackResult> {
    if profile.profile_id.is_empty() || profile.source_profile.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "time authority profile requires profile_id and source_profile".into(),
        ));
    }
    if profile.valid_until < profile.valid_from {
        return Ok(ValidateCallbackResult::Invalid(
            "time authority profile validity window is inverted".into(),
        ));
    }
    let machine_record = must_get_valid_record(profile.machine_hash.clone())?;
    if !matches!(machine_record.action(), Action::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "time authority profile must bind to a machine root".into(),
        ));
    }
    if machine_record.action().author() != action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "only the machine registrant may establish a time authority profile".into(),
        ));
    }
    if !verify_signature(
        action.author().clone(),
        profile.registrant_signature.clone(),
        profile.signed_payload(),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "time authority profile registrant signature is invalid".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_temporal_attestation(
    action: TypedAction<CreateData>,
    attestation: MachineTemporalAttestationEntry,
) -> ExternResult<ValidateCallbackResult> {
    if attestation.source_reference.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "temporal attestation requires a source reference".into(),
        ));
    }
    let profile_record = must_get_valid_record(attestation.profile_hash.clone())?;
    let profile: Option<MachineTimeAuthorityProfileEntry> = profile_record
        .entry().to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let Some(profile) = profile else {
        return Ok(ValidateCallbackResult::Invalid(
            "temporal attestation profile reference is not a time authority profile".into(),
        ));
    };
    if profile.machine_hash != attestation.machine_hash
        || profile.authority_agent != action.author()
        || !temporal_profile_contains(&profile, attestation.attested_at)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "temporal attestation is outside its exact machine/authority profile".into(),
        ));
    }
    let subject_record = must_get_valid_record(attestation.subject_hash.clone())?;
    match attestation.evidence_kind {
        MachineTemporalEvidenceKind::TransitionApproval => {
            let approval: Option<MachineControllerTransitionApprovalEntry> = subject_record
                .entry().to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
            let Some(approval) = approval else {
                return Ok(ValidateCallbackResult::Invalid(
                    "temporal approval evidence subject is not a transition approval".into(),
                ));
            };
            if approval.machine_hash != attestation.machine_hash {
                return Ok(ValidateCallbackResult::Invalid(
                    "temporal approval evidence subject belongs to a different machine".into(),
                ));
            }
        }
        MachineTemporalEvidenceKind::MachineActionExistence => {
            let machine: Option<MachineEntry> = subject_record
                .entry().to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
            let Some(_) = machine else {
                return Ok(ValidateCallbackResult::Invalid(
                    "machine action temporal evidence subject is not a machine record".into(),
                ));
            };
            let root = resolve_machine_root_action_hash(attestation.subject_hash.clone())?;
            if root != attestation.machine_hash {
                return Ok(ValidateCallbackResult::Invalid(
                    "machine action temporal evidence subject belongs to a different machine".into(),
                ));
            }
        }
    }
    if !verify_signature(
        profile.authority_agent.clone(),
        attestation.authority_signature.clone(),
        attestation.signed_payload(),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "temporal attestation authority signature is invalid".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_transition_approval(
    action: TypedAction<CreateData>,
    approval: MachineControllerTransitionApprovalEntry,
) -> ExternResult<ValidateCallbackResult> {
    if approval.valid_until < approval.valid_from {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval validity window is inverted".into(),
        ));
    }
    let Some(duration) = approval
        .valid_until
        .as_micros()
        .checked_sub(approval.valid_from.as_micros())
    else {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval validity arithmetic overflow".into(),
        ));
    };
    if duration > MAX_MACHINE_TRANSITION_APPROVAL_MICROS {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval exceeds the maximum 5-minute validity".into(),
        ));
    }

    let authority_record = must_get_valid_record(approval.authority_hash.clone())?;
    let authority: Option<MachineControllerAuthorityEntry> = authority_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let Some(authority) = authority else {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval references a non-authority record".into(),
        ));
    };
    if authority.lease_schema_version != 2 || !authority.requires_transition_approval {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval requires a schema-v2 controller lease".into(),
        ));
    }
    if authority.machine_hash != approval.machine_hash
        || authority.controller_agent != approval.controller_agent
    {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval does not match the controller authority".into(),
        ));
    }
    if approval.valid_from < authority.valid_from || approval.valid_until > authority.valid_until {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval interval exceeds controller lease interval".into(),
        ));
    }

    let machine_record = must_get_valid_record(approval.machine_hash.clone())?;
    let machine: Option<MachineEntry> = machine_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    if machine.is_none() || !matches!(machine_record.action(), Action::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval must target a machine root".into(),
        ));
    }
    if machine_record.action().author() != action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "only the machine registrant may create transition approvals".into(),
        ));
    }
    if !verify_signature(
        machine_record.action().author().clone(),
        approval.issuer_signature.clone(),
        approval.signed_payload(),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval issuer signature does not match the exact approval payload".into(),
        ));
    }

    let predecessor_record = must_get_valid_record(approval.predecessor_action.clone())?;
    if !matches!(predecessor_record.action(), Action::Create(_) | Action::Update(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval predecessor is not a machine entry action".into(),
        ));
    }
    let predecessor_machine: MachineEntry = predecessor_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "transition approval predecessor is not a machine record".into(),
        )))?;
    if !predecessor_machine.status.can_transition_to(&approval.new_status) {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval does not describe a valid machine transition".into(),
        ));
    }
    let predecessor_root = resolve_machine_root_action_hash(approval.predecessor_action.clone())?;
    if predecessor_root != approval.machine_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval predecessor belongs to a different machine".into(),
        ));
    }
    if !authority_valid_at(&authority, approval.valid_from)
        || !authority_valid_at(&authority, approval.valid_until)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval interval is outside controller authority validity".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_authority(
    action: TypedAction<CreateData>,
    authority: MachineControllerAuthorityEntry,
) -> ExternResult<ValidateCallbackResult> {
    if authority.lease_schema_version != 1 && authority.lease_schema_version != 2 {
        return Ok(ValidateCallbackResult::Invalid("unsupported machine controller lease schema version".into()));
    }
    if authority.lease_schema_version == 2 && !authority.requires_transition_approval {
        return Ok(ValidateCallbackResult::Invalid("lease schema v2 requires per-transition approval".into()));
    }
    if authority.valid_until < authority.valid_from {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority validity window is inverted".into(),
        ));
    }
    if !authority_duration_is_bounded(&authority) {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority exceeds the maximum 24-hour lease duration".into(),
        ));
    }
    let Some(signature) = authority.issuer_signature.clone() else {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority requires a registrant signature".into(),
        ));
    };

    let machine_record = must_get_valid_record(authority.machine_hash.clone())?;
    let machine: Option<MachineEntry> = machine_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    if machine.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority references a non-machine record".into(),
        ));
    }
    if !matches!(machine_record.action(), Action::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority must bind to the machine's root creation action".into(),
        ));
    }
    if machine_record.action().author() != action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "only the machine registrant may issue controller authority".into(),
        ));
    }
        let signature_valid = match authority.lease_schema_version {
        1 => verify_signature(
            action.author().clone(),
            signature,
            authority.signed_payload_v1(),
        )?,
        2 => verify_signature(
            action.author().clone(),
            signature,
            authority.signed_payload_v2(),
        )?,
        _ => false,
    };
    if !signature_valid {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority issuer signature does not match the lease payload".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_status_log(
    action: TypedAction<CreateData>,
    log: MachineStatusLog,
) -> ExternResult<ValidateCallbackResult> {
    if !log.previous_status.can_transition_to(&log.new_status) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid machine transition: {:?} -> {:?}",
            log.previous_status, log.new_status
        )));
    }

    let authority_record = must_get_valid_record(log.authority_hash.clone())?;
    let authority: Option<MachineControllerAuthorityEntry> = authority_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let Some(authority) = authority else {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status authority reference is not an authority record".into(),
        ));
    };

    if authority.machine_hash != log.machine_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status authority is bound to a different machine".into(),
        ));
    }
    if authority.issuer_signature.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "unsigned legacy controller authority cannot authorize new machine status".into(),
        ));
    }
    if authority.requires_transition_approval && log.transition_approval_hash.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "schema-v2 controller status requires a transition approval".into(),
        ));
    }
    if !authority.requires_transition_approval && log.transition_approval_hash.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "legacy controller status cannot carry a transition approval".into(),
        ));
    }
    if action.author() != authority.controller_agent {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status action author is not an authorized controller".into(),
        ));
    }
    if !authority_valid_at(&authority, action.timestamp()) {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status action falls outside controller authority validity".into(),
        ));
    }
    if log.changed_at != action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status changed_at must equal the action timestamp".into(),
        ));
    }

    let update_record = must_get_valid_record(log.machine_update_hash.clone())?;
    if !matches!(update_record.action(), Action::Update(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log must reference a machine update action".into(),
        ));
    }
    let updated_machine: Option<MachineEntry> = update_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let Some(updated_machine) = updated_machine else {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log update reference is not a machine record".into(),
        ));
    };

    let update_root = resolve_machine_root_action_hash(log.machine_update_hash.clone())?;
    if update_root != log.machine_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log update belongs to a different machine".into(),
        ));
    }
    if update_record.action().author() != action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log and machine update have different authors".into(),
        ));
    }
    if update_record.action().timestamp() > action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log cannot predate its referenced machine update".into(),
        ));
    }
    if updated_machine.status != log.new_status
        || updated_machine.current_work_order != log.work_order_hash
        || updated_machine.last_status_authority_hash != Some(log.authority_hash.clone())
        || (authority.requires_transition_approval
            && updated_machine.last_status_transition_approval_hash != log.transition_approval_hash)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log does not match its referenced machine update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_entry(entry: EntryTypes) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::Machine(m) => {
            if m.name.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "machine name is required".into(),
                ));
            }
            if m.location.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "machine location is required".into(),
                ));
            }
            if m.max_throughput_per_hour == 0 {
                return Ok(ValidateCallbackResult::Invalid(
                    "max_throughput_per_hour must be > 0".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::StatusLog(log) => {
            if !log.previous_status.can_transition_to(&log.new_status) {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Invalid machine transition: {:?} -> {:?}",
                    log.previous_status, log.new_status
                )));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::MachineControllerAuthority(_) => unreachable!(),
        EntryTypes::MachineControllerTransitionApproval(_) => unreachable!(),
        EntryTypes::MachineTimeAuthorityProfile(_) => unreachable!(),
        EntryTypes::MachineTemporalAttestation(_) => unreachable!(),
    }
}

fn resolve_machine_root_action_hash(
    start: ActionHash,
) -> ExternResult<ActionHash> {
    let mut current = start;
    for _ in 0..4096 {
        let record = must_get_valid_record(current.clone())?;
        match record.action() {
            Action::Create(_) => {
                let root_record = must_get_valid_record(current.clone())?;
                let root_machine: Option<MachineEntry> = root_record
                    .entry()
                    .to_app_option()
                    .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
                if root_machine.is_none() {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "machine update lineage terminates at a non-machine root".into(),
                    )));
                }
                return Ok(current);
            }
            Action::Update(update) => {
                current = update.original_action_address.clone();
            }
            _ => {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "machine update lineage terminated at a non-entry action".into(),
                )));
            }
        }
    }

    Err(wasm_error!(WasmErrorInner::Guest(
        "machine update lineage exceeds safety bound".into(),
    )))
}

fn machine_control_fields_changed(
    original: &MachineEntry,
    updated: &MachineEntry,
) -> bool {
    original.status != updated.status
        || original.current_work_order != updated.current_work_order
        || original.last_status_authority_hash != updated.last_status_authority_hash
        || original.last_status_transition_approval_hash != updated.last_status_transition_approval_hash
}

fn transition_approval_matches_update(
    approval: &MachineControllerTransitionApprovalEntry,
    machine_root: &ActionHash,
    authority_hash: &ActionHash,
    controller_agent: &AgentPubKey,
    predecessor_action: &ActionHash,
    new_status: &MachineStatus,
    work_order_hash: Option<&ActionHash>,
    action_timestamp: Timestamp,
) -> bool {
    approval.machine_hash == *machine_root
        && approval.authority_hash == *authority_hash
        && approval.controller_agent == *controller_agent
        && approval.predecessor_action == *predecessor_action
        && approval.new_status == *new_status
        && approval.work_order_hash.as_ref() == work_order_hash
        && approval_valid_at(approval, action_timestamp)
}
fn temporal_interval_contains(
    valid_from: Timestamp,
    valid_until: Timestamp,
    timestamp: Timestamp,
) -> bool {
    valid_from <= timestamp && timestamp <= valid_until
}

fn temporal_profile_contains(
    profile: &MachineTimeAuthorityProfileEntry,
    timestamp: Timestamp,
) -> bool {
    temporal_interval_contains(profile.valid_from, profile.valid_until, timestamp)
}
fn authority_valid_at(
    authority: &MachineControllerAuthorityEntry,
    timestamp: Timestamp,
) -> bool {
    authority.valid_from <= timestamp && timestamp <= authority.valid_until
}

fn authority_duration_is_bounded(
    authority: &MachineControllerAuthorityEntry,
) -> bool {
    let Some(duration) = authority
        .valid_until
        .as_micros()
        .checked_sub(authority.valid_from.as_micros())
    else {
        return false;
    };
    duration <= MAX_MACHINE_CONTROLLER_LEASE_MICROS
}

fn validate_update_entry(
    original_action_hash: ActionHash,
    action: TypedAction<UpdateData>,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::Machine(m) => {
            let original_record = must_get_valid_record(original_action_hash)?;
            let original: MachineEntry = original_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Original machine not found".into()
                )))?;

            if m.name != original.name
                || m.machine_type != original.machine_type
                || m.capabilities != original.capabilities
                || m.location != original.location
                || m.max_throughput_per_hour != original.max_throughput_per_hour
                || m.registered_at != original.registered_at
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only status/current_work_order/last_status_authority_hash/last_status_transition_approval_hash can change on a machine update".into(),
                ));
            }

            if machine_control_fields_changed(&original, &m) {
                let Some(authority_hash) = m.last_status_authority_hash.clone() else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine control-field changes require controller authority".into(),
                    ));
                };
                let authority_record = must_get_valid_record(authority_hash)?;
                let authority: Option<MachineControllerAuthorityEntry> = authority_record
                    .entry()
                    .to_app_option()
                    .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
                let Some(authority) = authority else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine status authority reference is not an authority record".into(),
                    ));
                };

                let machine_root = resolve_machine_root_action_hash(
                    original_action_hash.clone(),
                )?;
                if authority.machine_hash != machine_root {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine status authority is bound to a different machine".into(),
                    ));
                }
                if authority.issuer_signature.is_none() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "unsigned legacy controller authority cannot authorize new machine updates".into(),
                    ));
                }
                if !authority.requires_transition_approval
                    && m.last_status_transition_approval_hash.is_some()
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "legacy controller update cannot carry a transition approval".into(),
                    ));
                }
                if authority.requires_transition_approval {
                    let Some(approval_hash) = m.last_status_transition_approval_hash.clone() else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "schema-v2 controller updates require a transition approval".into(),
                        ));
                    };
                    let approval_record = must_get_valid_record(approval_hash)?;
                    let approval: Option<MachineControllerTransitionApprovalEntry> = approval_record
                        .entry().to_app_option()
                        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
                    let Some(approval) = approval else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "transition approval reference is not an approval record".into(),
                        ));
                    };
                    if approval.valid_from < authority.valid_from
                        || approval.valid_until > authority.valid_until
                        || !transition_approval_matches_update(
                            &approval,
                            &machine_root,
                            &authority_hash,
                            action.author(),
                            &original_action_hash,
                            &m.status,
                            m.current_work_order.as_ref(),
                            action.timestamp(),
                        )
                    {
                        return Ok(ValidateCallbackResult::Invalid(
                            "transition approval does not exactly authorize this machine update".into(),
                        ));
                    }
                    let machine_root_record = must_get_valid_record(machine_root.clone())?;
                    if !verify_signature(
                        machine_root_record.action().author().clone(),
                        approval.issuer_signature.clone(),
                        approval.signed_payload(),
                    )? {
                        return Ok(ValidateCallbackResult::Invalid(
                            "transition approval signature is invalid for this machine registrant".into(),
                        ));
                    }
                }
                if action.author() != authority.controller_agent {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine update author is not an authorized controller".into(),
                    ));
                }
                if !authority_valid_at(&authority, action.timestamp()) {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine update falls outside controller authority validity".into(),
                    ));
                }
                if m.status != original.status
                    && !original.status.can_transition_to(&m.status)
                {
                    return Ok(ValidateCallbackResult::Invalid(format!(
                        "Invalid machine transition: {:?} -> {:?}",
                        original.status, m.status
                    )));
                }
            }

            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::StatusLog(_) => Ok(ValidateCallbackResult::Invalid(
            "Status log records are immutable".into(),
        )),
        EntryTypes::MachineControllerAuthority(_) => Ok(ValidateCallbackResult::Invalid(
            "Machine controller authorities are immutable".into(),
        )),
    }
}

#[cfg(test)]
mod content_restriction_tests {
    use super::*;

    fn valid_machine() -> MachineEntry {
        MachineEntry {
            name: "Mill-1".into(),
            machine_type: MachineType::CNC3Axis,
            capabilities: vec![],
            location: "Bay 1".into(),
            max_throughput_per_hour: 10,
            status: MachineStatus::Available,
            current_work_order: None,
            last_status_authority_hash: None,
            last_status_transition_approval_hash: None,
            registered_at: Timestamp::from_micros(0),
        }
    }

    fn sample_approval() -> MachineControllerTransitionApprovalEntry {
        let valid_from = Timestamp::from_micros(100);
        MachineControllerTransitionApprovalEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            authority_hash: ActionHash::from_raw_36(vec![2; 36]),
            controller_agent: AgentPubKey::from_raw_32(vec![3; 32]),
            predecessor_action: ActionHash::from_raw_36(vec![4; 36]),
            new_status: MachineStatus::Running,
            work_order_hash: Some(ActionHash::from_raw_36(vec![5; 36])),
            valid_from,
            valid_until: Timestamp::from_micros(200),
            issuer_signature: Signature(vec![0; 64]),
        }
    }

    #[test]
    fn transition_approval_binds_exact_update_tuple() {
        let approval = sample_approval();
        let machine = ActionHash::from_raw_36(vec![1; 36]);
        let authority = ActionHash::from_raw_36(vec![2; 36]);
        let controller = AgentPubKey::from_raw_32(vec![3; 32]);
        let predecessor = ActionHash::from_raw_36(vec![4; 36]);
        let work_order = ActionHash::from_raw_36(vec![5; 36]);
        assert!(transition_approval_matches_update(
            &approval, &machine, &authority, &controller, &predecessor,
            &MachineStatus::Running, Some(&work_order), Timestamp::from_micros(150)
        ));

        let wrong_machine = ActionHash::from_raw_36(vec![9; 36]);
        assert!(!transition_approval_matches_update(
            &approval, &wrong_machine, &authority, &controller, &predecessor,
            &MachineStatus::Running, Some(&work_order), Timestamp::from_micros(150)
        ));

        let wrong_predecessor = ActionHash::from_raw_36(vec![8; 36]);
        assert!(!transition_approval_matches_update(
            &approval, &machine, &authority, &controller, &wrong_predecessor,
            &MachineStatus::Running, Some(&work_order), Timestamp::from_micros(150)
        ));

        assert!(!transition_approval_matches_update(
            &approval, &machine, &authority, &controller, &predecessor,
            &MachineStatus::Maintenance, Some(&work_order), Timestamp::from_micros(150)
        ));

        assert!(!transition_approval_matches_update(
            &approval, &machine, &authority, &controller, &predecessor,
            &MachineStatus::Running, Some(&work_order), Timestamp::from_micros(201)
        ));
    }
    #[test]
    fn temporal_evidence_conflicts_fail_closed() {
        assert_eq!(resolve_temporal_times(vec![]), MachineTemporalEvidenceResolution::NoEvidence);
        assert_eq!(resolve_temporal_times(vec![Timestamp::from_micros(100), Timestamp::from_micros(100)]), MachineTemporalEvidenceResolution::Unique(Timestamp::from_micros(100)));
        assert_eq!(resolve_temporal_times(vec![Timestamp::from_micros(100), Timestamp::from_micros(200)]), MachineTemporalEvidenceResolution::Conflicting(vec![Timestamp::from_micros(100), Timestamp::from_micros(200)]));
    }
    #[test]
    fn temporal_profile_boundaries_are_inclusive() {
        let from = Timestamp::from_micros(100);
        let until = Timestamp::from_micros(200);
        assert!(temporal_interval_contains(from, until, Timestamp::from_micros(100)));
        assert!(temporal_interval_contains(from, until, Timestamp::from_micros(200)));
        assert!(!temporal_interval_contains(from, until, Timestamp::from_micros(99)));
        assert!(!temporal_interval_contains(from, until, Timestamp::from_micros(201)));
    }
    #[test]
    fn controller_lease_duration_is_bounded() {
        let start = Timestamp::from_micros(1_000_000);
        let within = MachineControllerAuthorityEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            controller_agent: AgentPubKey::from_raw_32(vec![2; 32]),
            valid_from: start,
            valid_until: Timestamp::from_micros(
                start.as_micros() + MAX_MACHINE_CONTROLLER_LEASE_MICROS,
            ),
            issuer_signature: None,
            lease_schema_version: 1,
            requires_transition_approval: false,
        };
        assert!(authority_duration_is_bounded(&within));

        let beyond = MachineControllerAuthorityEntry {
            valid_until: Timestamp::from_micros(
                start.as_micros() + MAX_MACHINE_CONTROLLER_LEASE_MICROS + 1,
            ),
            ..within.clone()
        };
        assert!(!authority_duration_is_bounded(&beyond));
    }

    #[test]
    fn authority_validity_is_inclusive() {
        let authority = MachineControllerAuthorityEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            controller_agent: AgentPubKey::from_raw_32(vec![2; 32]),
            valid_from: Timestamp::from_micros(100),
            valid_until: Timestamp::from_micros(200),
            lease_schema_version: 1,
            requires_transition_approval: false,
            issuer_signature: None,
        };

        assert!(authority_valid_at(&authority, Timestamp::from_micros(100)));
        assert!(authority_valid_at(&authority, Timestamp::from_micros(200)));
        assert!(!authority_valid_at(&authority, Timestamp::from_micros(99)));
        assert!(!authority_valid_at(&authority, Timestamp::from_micros(201)));
    }

    #[test]
    fn transition_approval_interval_is_bounded() {
        let start = Timestamp::from_micros(10_000);
        let within = MachineControllerTransitionApprovalEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            authority_hash: ActionHash::from_raw_36(vec![2; 36]),
            controller_agent: AgentPubKey::from_raw_32(vec![3; 32]),
            predecessor_action: ActionHash::from_raw_36(vec![4; 36]),
            new_status: MachineStatus::Running,
            work_order_hash: None,
            valid_from: start,
            valid_until: Timestamp::from_micros(start.as_micros() + MAX_MACHINE_TRANSITION_APPROVAL_MICROS),
            issuer_signature: Signature(vec![0; 64]),
        };
        assert!(approval_valid_at(&within, Timestamp::from_micros(10_000)));
        assert!(approval_valid_at(&within, Timestamp::from_micros(10_000 + MAX_MACHINE_TRANSITION_APPROVAL_MICROS)));
        assert!(!approval_valid_at(&within, Timestamp::from_micros(9_999)));
        assert!(!approval_valid_at(&within, Timestamp::from_micros(10_000 + MAX_MACHINE_TRANSITION_APPROVAL_MICROS + 1)));
    }
    #[test]
    fn control_field_change_requires_authority() {
        let original = valid_machine();
        let mut updated = original.clone();

        updated.current_work_order = Some(ActionHash::from_raw_36(vec![7; 36]));
        assert!(machine_control_fields_changed(&original, &updated));

        updated = original.clone();
        updated.last_status_authority_hash = Some(ActionHash::from_raw_36(vec![8; 36]));
        assert!(machine_control_fields_changed(&original, &updated));

        updated = original.clone();
        updated.status = MachineStatus::Running;
        assert!(machine_control_fields_changed(&original, &updated));

        assert!(!machine_control_fields_changed(&original, &original));
    }

    #[test]
    fn create_machine_requires_name_and_location() {
        let mut m = valid_machine();
        m.name = "".into();
        let result = validate_create_entry(EntryTypes::Machine(m)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_machine_requires_positive_throughput() {
        let mut m = valid_machine();
        m.max_throughput_per_hour = 0;
        let result = validate_create_entry(EntryTypes::Machine(m)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_machine_valid() {
        let result = validate_create_entry(EntryTypes::Machine(valid_machine())).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn status_log_rejects_invalid_transition() {
        let log = MachineStatusLog {
            machine_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            machine_update_hash: ActionHash::from_raw_36(vec![8u8; 36]),
            authority_hash: ActionHash::from_raw_36(vec![9u8; 36]),
            previous_status: MachineStatus::Offline,
            new_status: MachineStatus::Running,
            work_order_hash: None,
            transition_approval_hash: None,
            changed_at: Timestamp::from_micros(0),
        };
        let result = validate_create_entry(EntryTypes::StatusLog(log)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // validate_update_entry calls must_get_valid_record, which requires a
    // live HDI host and can't run in a plain unit test -- matching the
    // established pattern from every other zome's update validator this
    // pass.
}
