// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Machines Coordinator Zome
//!
//! Machine registry and status management for the manufacturing floor.

use hdk::prelude::*;
use machines_integrity::*;
use manufacturing_common::{MachineStatus, MachineType};

// ============================================================================
// Input types
// ============================================================================

#[derive(Serialize, Deserialize, Debug)]
pub struct RegisterMachineInput {
    pub name: String,
    pub machine_type: MachineType,
    pub capabilities: Vec<String>,
    pub location: String,
    pub max_throughput_per_hour: u32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct GrantMachineControllerInput {
    pub machine_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct UpdateMachineStatusInput {
    pub machine_hash: ActionHash,
    pub new_status: MachineStatus,
    pub work_order_hash: Option<ActionHash>,
    pub authority_hash: ActionHash,
    #[serde(default)]
    pub transition_approval_hash: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateMachineTransitionApprovalInput {
    pub machine_hash: ActionHash,
    pub authority_hash: ActionHash,
    pub new_status: MachineStatus,
    pub work_order_hash: Option<ActionHash>,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateMachineTimeAuthorityProfileInput {
    pub machine_hash: ActionHash,
    pub authority_agent: AgentPubKey,
    pub profile_id: String,
    pub source_profile: String,
    pub source_authority_commitment: Vec<u8>,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub max_accuracy_micros: i64,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateMachineTemporalAttestationInput {
    pub machine_hash: ActionHash,
    pub profile_hash: ActionHash,
    pub subject_hash: ActionHash,
    pub evidence_kind: MachineTemporalEvidenceKind,
    pub attested_at: Timestamp,
    pub accuracy_micros: i64,
    pub source_reference: String,
    pub source_commitment: Vec<u8>,
}
#[derive(Serialize, Deserialize, Debug)]
pub struct GetMachinesByTypeInput {
    pub machine_type: MachineType,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MachineStateResolution {
    NotFound,
    InvalidRecord,
    Resolved {
        status: MachineStatus,
        head_action: ActionHash,
    },
    Ambiguous {
        statuses: Vec<MachineStatus>,
        head_actions: Vec<ActionHash>,
    },
    Deleted,
}

/// Resolve the machine's current state from the full update graph rooted at
/// the original machine action.
///
/// Holochain does not define a global "latest" record. An action-addressed
/// get_details call exposes direct updates, while entry-level metadata exposes
/// the relationships for the entry. We therefore walk update edges to terminal
/// live heads and refuse to select among multiple heads.
fn resolve_machine_state_from_action(
    root: ActionHash,
) -> ExternResult<MachineStateResolution> {
    let mut pending = vec![root];
    let mut visited = std::collections::HashSet::new();
    let mut heads: Vec<(ActionHash, MachineStatus)> = Vec::new();
    let mut saw_deleted = false;

    while let Some(action_hash) = pending.pop() {
        if !visited.insert(action_hash.clone()) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "machine update graph contains a cycle".into(),
            )));
        }
        if visited.len() > 4096 {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "machine update graph exceeds safety bound".into(),
            )));
        }

        let details = get_details(action_hash.clone(), GetOptions::default())?;
        let Some(Details::Record(record_details)) = details else {
            return Ok(MachineStateResolution::NotFound);
        };

        if record_details.validation_status != ValidationStatus::Valid {
            return Ok(MachineStateResolution::InvalidRecord);
        }

        if !record_details.deletes.is_empty() {
            saw_deleted = true;
        }

        if !record_details.updates.is_empty() {
            for update in record_details.updates {
                pending.push(update.as_hash().clone());
            }
            continue;
        }

        if !record_details.deletes.is_empty() {
            continue;
        }

        let machine: MachineEntry = record_details
            .record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "machine state head has no entry".into(),
            )))?;

        heads.push((action_hash, machine.status));
    }

    if heads.is_empty() {
        return Ok(if saw_deleted {
            MachineStateResolution::Deleted
        } else {
            MachineStateResolution::NotFound
        });
    }

    if heads.len() == 1 {
        let (head_action, status) = heads.remove(0);
        return Ok(MachineStateResolution::Resolved {
            status,
            head_action,
        });
    }

    heads.sort_by(|a, b| a.0.to_string().cmp(&b.0.to_string()));
    Ok(MachineStateResolution::Ambiguous {
        statuses: heads.iter().map(|(_, status)| status.clone()).collect(),
        head_actions: heads.into_iter().map(|(hash, _)| hash).collect(),
    })
}

#[hdk_extern]
pub fn get_current_machine_state(machine_hash: ActionHash) -> ExternResult<MachineStateResolution> {
    resolve_machine_state_from_action(machine_hash)
}

/// Grant a controller a time-scoped authority over a machine's status.
#[hdk_extern]
pub fn grant_machine_controller(
    input: GrantMachineControllerInput,
) -> ExternResult<ActionHash> {
    let machine_record = get(input.machine_hash.clone(), GetOptions::default())?.ok_or(
        wasm_error!(WasmErrorInner::Guest("Machine not found".into())),
    )?;
    let machine: MachineEntry = machine_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Could not deserialize machine".into(),
        )))?;

    if !matches!(machine_record.action(), Action::Create(_)) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "controller authority must target the machine's root creation action".into(),
        )));
    }
    if machine_record.action().author() != agent_info()?.agent_initial_pubkey {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "only the machine registrant may grant controller authority".into(),
        )));
    }
    if input.valid_until < input.valid_from {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "controller authority validity window is inverted".into(),
        )));
    }

    let issuer = machine_record.action().author().clone();
    let authority = MachineControllerAuthorityEntry {
        machine_hash: input.machine_hash.clone(),
        controller_agent: input.controller_agent,
        valid_from: input.valid_from,
        valid_until: input.valid_until,
        lease_schema_version: 2,
        requires_transition_approval: true,
        issuer_signature: None,
    };
    let issuer_signature = sign(issuer, authority.signed_payload_v2())?;
    let authority = MachineControllerAuthorityEntry {
        issuer_signature: Some(issuer_signature),
        ..authority
    };
    let hash = create_entry(EntryTypes::MachineControllerAuthority(authority))?;

    create_link(
        input.machine_hash.clone(),
        hash.clone(),
        LinkTypes::MachineToAuthorities,
        (),
    )?;

    let all_path = Path::from("all_machine_controller_authorities")
        .typed(LinkTypes::AllMachineControllerAuthorities)?;
    all_path.ensure()?;
    create_link(
        all_path.path_entry_hash()?,
        hash.clone(),
        LinkTypes::AllMachineControllerAuthorities,
        (),
    )?;

    let _ = machine;
    Ok(hash)
}

/// Create a registrant-signed authorization for one exact machine transition.
#[hdk_extern]
pub fn create_machine_transition_approval(
    input: CreateMachineTransitionApprovalInput,
) -> ExternResult<ActionHash> {
    let machine_record = get(input.machine_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Machine not found".into())))?;
    if !matches!(machine_record.action(), Action::Create(_)) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "transition approval must target the machine root".into(),
        )));
    }
    if machine_record.action().author() != agent_info()?.agent_initial_pubkey {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "only the machine registrant may create transition approvals".into(),
        )));
    }
    let MachineStateResolution::Resolved { head_action, .. } =
        get_current_machine_state(input.machine_hash.clone())?
    else {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "machine state is not uniquely resolvable".into(),
        )));
    };
    let head_record = get(head_action.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Current machine head not found".into())))?;
    let current_machine: MachineEntry = head_record.entry().to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Could not deserialize current machine".into())))?;
    if !current_machine.status.can_transition_to(&input.new_status) {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid machine transition: {:?} -> {:?}", current_machine.status, input.new_status
        ))));
    }
    let authority_record = get(input.authority_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Machine controller authority not found".into())))?;
    let authority: MachineControllerAuthorityEntry = authority_record.entry().to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Could not deserialize machine controller authority".into())))?;
    if authority.lease_schema_version != 2 || !authority.requires_transition_approval {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "transition approval requires a schema-v2 controller lease".into(),
        )));
    }
    if authority.issuer_signature.is_none() || authority.machine_hash != input.machine_hash {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "transition approval authority provenance is invalid".into(),
        )));
    }
    if input.valid_until < input.valid_from {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "transition approval validity window is inverted".into(),
        )));
    }
    let duration = input.valid_until.as_micros().checked_sub(input.valid_from.as_micros())
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "transition approval validity arithmetic overflow".into(),
        )))?;
    if duration > MAX_MACHINE_TRANSITION_APPROVAL_MICROS {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "transition approval exceeds the maximum 5-minute validity".into(),
        )));
    }
    if input.valid_from < authority.valid_from || input.valid_until > authority.valid_until {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "transition approval interval exceeds controller authority interval".into(),
        )));
    }
    let approval = MachineControllerTransitionApprovalEntry {
        machine_hash: input.machine_hash.clone(),
        authority_hash: input.authority_hash.clone(),
        controller_agent: authority.controller_agent.clone(),
        predecessor_action: head_action.clone(),
        new_status: input.new_status.clone(),
        work_order_hash: input.work_order_hash.clone(),
        valid_from: input.valid_from,
        valid_until: input.valid_until,
        issuer_signature: sign(
            machine_record.action().author().clone(),
            MachineControllerTransitionApprovalPayload {
                schema_id: MACHINE_CONTROLLER_TRANSITION_APPROVAL_SCHEMA_ID.to_string(),
                machine_hash: input.machine_hash.clone(),
                authority_hash: input.authority_hash.clone(),
                controller_agent: authority.controller_agent.clone(),
                predecessor_action: head_action.clone(),
                new_status: input.new_status.clone(),
                work_order_hash: input.work_order_hash.clone(),
                valid_from: input.valid_from,
                valid_until: input.valid_until,
            },
        )?,
    };
    let hash = create_entry(EntryTypes::MachineControllerTransitionApproval(approval))?;
    create_link(input.machine_hash.clone(), hash.clone(), LinkTypes::MachineToTransitionApprovals, ())?;
    let path = Path::from("all_machine_transition_approvals")
        .typed(LinkTypes::AllMachineTransitionApprovals)?;
    path.ensure()?;
    create_link(path.path_entry_hash()?, hash.clone(), LinkTypes::AllMachineTransitionApprovals, ())?;
    Ok(hash)
}
/// Register a registrant-approved source/policy for temporal evidence on one machine.
#[hdk_extern]
pub fn create_machine_time_authority_profile(
    input: CreateMachineTimeAuthorityProfileInput,
) -> ExternResult<ActionHash> {
    let machine_record = get(input.machine_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Machine not found".into())))?;
    if !matches!(machine_record.action(), Action::Create(_)) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "time authority profile must target the machine root".into(),
        )));
    }
    if machine_record.action().author() != agent_info()?.agent_initial_pubkey {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "only the machine registrant may create a time authority profile".into(),
        )));
    }
    if input.valid_until < input.valid_from
        || input.profile_id.is_empty()
        || input.source_profile.is_empty()
        || input.source_authority_commitment.is_empty()
        || input.source_authority_commitment.len() > MAX_MACHINE_TEMPORAL_SOURCE_COMMITMENT_BYTES
        || input.max_accuracy_micros < 0
        || input.max_accuracy_micros > MAX_MACHINE_TEMPORAL_ACCURACY_MICROS
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "time authority profile has invalid identity or accuracy bounds".into(),
        )));
    }
    let profile_payload = MachineTimeAuthorityProfilePayload {
        schema_id: MACHINE_TIME_AUTHORITY_PROFILE_SCHEMA_ID.to_string(),
        machine_hash: input.machine_hash.clone(),
        authority_agent: input.authority_agent.clone(),
        profile_id: input.profile_id.clone(),
        source_profile: input.source_profile.clone(),
        source_authority_commitment: input.source_authority_commitment.clone(),
        valid_from: input.valid_from,
        valid_until: input.valid_until,
        max_accuracy_micros: input.max_accuracy_micros,
    };
    let profile_signature = sign(machine_record.action().author().clone(), profile_payload)?;
    let profile = MachineTimeAuthorityProfileEntry {
        machine_hash: input.machine_hash.clone(),
        authority_agent: input.authority_agent,
        profile_id: input.profile_id,
        source_profile: input.source_profile,
        source_authority_commitment: input.source_authority_commitment,
        valid_from: input.valid_from,
        valid_until: input.valid_until,
        max_accuracy_micros: input.max_accuracy_micros,
        registrant_signature: profile_signature,
    };
    let hash = create_entry(EntryTypes::MachineTimeAuthorityProfile(profile))?;
    create_link(input.machine_hash.clone(), hash.clone(), LinkTypes::MachineToTimeAuthorityProfiles, ())?;
    let path = Path::from("all_machine_time_authority_profiles")
        .typed(LinkTypes::AllMachineTimeAuthorityProfiles)?;
    path.ensure()?;
    create_link(path.path_entry_hash()?, hash.clone(), LinkTypes::AllMachineTimeAuthorityProfiles, ())?;
    Ok(hash)
}

/// Record an authority-signed temporal attestation for an exact immutable subject.
#[hdk_extern]
pub fn create_machine_temporal_attestation(
    input: CreateMachineTemporalAttestationInput,
) -> ExternResult<ActionHash> {
    let profile_record = get(input.profile_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Time authority profile not found".into())))?;
    let profile: MachineTimeAuthorityProfileEntry = profile_record.entry().to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Could not deserialize time authority profile".into())))?;
    if profile.machine_hash != input.machine_hash {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "temporal attestation profile is bound to a different machine".into(),
        )));
    }
    if profile.authority_agent != agent_info()?.agent_initial_pubkey {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "current agent is not the designated time authority".into(),
        )));
    }
    if !temporal_profile_contains_interval(
            &profile,
            input.attested_at,
            input.accuracy_micros,
        )
        || input.source_reference.is_empty()
        || input.source_commitment.is_empty()
        || input.source_commitment.len() > MAX_MACHINE_TEMPORAL_SOURCE_COMMITMENT_BYTES
        || input.accuracy_micros < 0
        || input.accuracy_micros > MAX_MACHINE_TEMPORAL_ACCURACY_MICROS
        || input.accuracy_micros > profile.max_accuracy_micros
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "temporal attestation is outside profile validity or lacks bounded source evidence".into(),
        )));
    }
    let attestation_payload = MachineTemporalAttestationPayload {
        schema_id: MACHINE_TEMPORAL_ATTESTATION_SCHEMA_ID.to_string(),
        machine_hash: input.machine_hash.clone(),
        profile_hash: input.profile_hash.clone(),
        subject_hash: input.subject_hash.clone(),
        evidence_kind: input.evidence_kind.clone(),
        attested_at: input.attested_at,
        accuracy_micros: input.accuracy_micros,
        source_reference: input.source_reference.clone(),
        source_commitment: input.source_commitment.clone(),
    };
    let attestation_signature = sign(profile.authority_agent.clone(), attestation_payload)?;
    let attestation = MachineTemporalAttestationEntry {
        machine_hash: input.machine_hash.clone(),
        profile_hash: input.profile_hash,
        subject_hash: input.subject_hash,
        evidence_kind: input.evidence_kind,
        attested_at: input.attested_at,
        accuracy_micros: input.accuracy_micros,
        source_reference: input.source_reference,
        source_commitment: input.source_commitment,
        authority_signature: attestation_signature,
    };
    let hash = create_entry(EntryTypes::MachineTemporalAttestation(attestation))?;
    create_link(input.machine_hash.clone(), hash.clone(), LinkTypes::MachineToTemporalAttestations, ())?;
    create_link(input.subject_hash.clone(), hash.clone(), LinkTypes::SubjectToTemporalAttestations, ())?;
    let path = Path::from("all_machine_temporal_attestations")
        .typed(LinkTypes::AllMachineTemporalAttestations)?;
    path.ensure()?;
    create_link(path.path_entry_hash()?, hash.clone(), LinkTypes::AllMachineTemporalAttestations, ())?;
    Ok(hash)
}
/// Fetch one temporal attestation by action hash.
#[hdk_extern]
pub fn get_machine_temporal_attestation(hash: ActionHash) -> ExternResult<Option<Record>> {
    get(hash, GetOptions::default())
}

/// Resolve all valid temporal attestations for an exact subject.
///
/// Resolution is fail-closed and provenance-preserving: agreeing authorities remain
/// visible in a Unique result, while distinct attested times remain Conflicting.
#[hdk_extern]
pub fn resolve_machine_temporal_attestations(
    subject_hash: ActionHash,
) -> ExternResult<MachineTemporalEvidenceResolution> {
    let links = get_links(
        GetLinksInputBuilder::try_new(
            subject_hash.clone(),
            LinkTypes::SubjectToTemporalAttestations,
        )?
        .build(),
    )?;
    let mut evidence = Vec::with_capacity(links.len());
    for link in links {
        let Some(hash) = link.target.into_action_hash() else {
            return Ok(MachineTemporalEvidenceResolution::InvalidEvidence);
        };
        let Some(Details::Record(record_details)) = get_details(hash.clone(), GetOptions::default())? else {
            return Ok(MachineTemporalEvidenceResolution::InvalidEvidence);
        };
        if record_details.validation_status != ValidationStatus::Valid {
            return Ok(MachineTemporalEvidenceResolution::InvalidEvidence);
        }

        let record = record_details.record;
        let Some(attestation): Option<MachineTemporalAttestationEntry> = record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        else {
            return Ok(MachineTemporalEvidenceResolution::InvalidEvidence);
        };
        if attestation.subject_hash != subject_hash {
            return Ok(MachineTemporalEvidenceResolution::InvalidEvidence);
        }

        let Some(Details::Record(profile_details)) =
            get_details(attestation.profile_hash.clone(), GetOptions::default())?
        else {
            return Ok(MachineTemporalEvidenceResolution::InvalidEvidence);
        };
        if profile_details.validation_status != ValidationStatus::Valid {
            return Ok(MachineTemporalEvidenceResolution::InvalidEvidence);
        }
        let Some(profile_record) = profile_details.record.entry().to_app_option::<MachineTimeAuthorityProfileEntry>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        else {
            return Ok(MachineTemporalEvidenceResolution::InvalidEvidence);
        };

        if profile_record.machine_hash != attestation.machine_hash
            || profile_record.authority_agent != record.action().author()
            || attestation.accuracy_micros > profile_record.max_accuracy_micros
            || !temporal_interval_contains_interval(
                profile_record.valid_from,
                profile_record.valid_until,
                attestation.attested_at,
                attestation.accuracy_micros,
            )
        {
            return Ok(MachineTemporalEvidenceResolution::InvalidEvidence);
        }

        evidence.push(MachineTemporalEvidenceObservation {
            attestation_hash: hash,
            authority_agent: profile_record.authority_agent,
            profile_hash: attestation.profile_hash,
            subject_hash: attestation.subject_hash,
            evidence_kind: attestation.evidence_kind,
            attested_at: attestation.attested_at,
            accuracy_micros: attestation.accuracy_micros,
            source_reference: attestation.source_reference,
            source_commitment: attestation.source_commitment,
        });
    }

    Ok(resolve_temporal_evidence(evidence))
}

/// Get a controller authority by action hash.
#[hdk_extern]
pub fn get_machine_controller_authority(
    hash: ActionHash,
) -> ExternResult<Option<Record>> {
    get(hash, GetOptions::default())
}

/// List controller authorities granted for a machine.
#[hdk_extern]
pub fn list_machine_controller_authorities(
    machine_hash: ActionHash,
) -> ExternResult<Vec<Link>> {
    get_links(
        GetLinksInputBuilder::try_new(
            machine_hash,
            LinkTypes::MachineToAuthorities,
        )?
        .build(),
    )
}

/// List every controller authority in the machine registry.
#[hdk_extern]
pub fn list_all_machine_controller_authorities(_: ()) -> ExternResult<Vec<Link>> {
    let path = Path::from("all_machine_controller_authorities")
        .typed(LinkTypes::AllMachineControllerAuthorities)?;
    get_links(
        GetLinksInputBuilder::try_new(
            path.path_entry_hash()?,
            LinkTypes::AllMachineControllerAuthorities,
        )?
        .build(),
    )
}

/// Update machine status (e.g., Available -> Running).
#[hdk_extern]
pub fn update_machine_status(input: UpdateMachineStatusInput) -> ExternResult<ActionHash> {
    let resolution = get_current_machine_state(input.machine_hash.clone())?;
    let head_action = match resolution {
        MachineStateResolution::Resolved { status, head_action } => {
            if !status.can_transition_to(&input.new_status) {
                return Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "Invalid machine transition: {:?} -> {:?}",
                    status, input.new_status
                ))));
            }
            head_action
        }
        MachineStateResolution::NotFound => {
            return Err(wasm_error!(WasmErrorInner::Guest("Machine not found".into())));
        }
        MachineStateResolution::InvalidRecord => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Machine state contains an invalid record".into(),
            )));
        }
        MachineStateResolution::Ambiguous { .. } => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Machine state is ambiguous; status update refused".into(),
            )));
        }
        MachineStateResolution::Deleted => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Machine is deleted".into(),
            )));
        }
    };

    let record = get(head_action.clone(), GetOptions::default())?.ok_or(
        wasm_error!(WasmErrorInner::Guest("Current machine head not found".into())),
    )?;
    let machine: MachineEntry = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Could not deserialize current machine head".into(),
        )))?;

    let authority_record =
        get(input.authority_hash.clone(), GetOptions::default())?.ok_or(
            wasm_error!(WasmErrorInner::Guest(
                "Machine controller authority not found".into(),
            )),
        )?;
    let authority: MachineControllerAuthorityEntry = authority_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Could not deserialize machine controller authority".into(),
        )))?;

    let now = sys_time()?;
    if authority.machine_hash != input.machine_hash {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Controller authority is bound to a different machine".into(),
        )));
    }
    if authority.controller_agent != agent_info()?.agent_initial_pubkey {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Current agent is not the authorized machine controller".into(),
        )));
    }
    if authority.issuer_signature.is_none() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Unsigned legacy controller authority cannot authorize new machine status".into(),
        )));
    }
    if authority.requires_transition_approval {
        let approval_hash = input.transition_approval_hash.clone().ok_or(wasm_error!(
            WasmErrorInner::Guest("schema-v2 controller status requires a transition approval".into()),
        ))?;
        let approval_record = get(approval_hash.clone(), GetOptions::default())?
            .ok_or(wasm_error!(WasmErrorInner::Guest("Transition approval not found".into())))?;
        let approval: MachineControllerTransitionApprovalEntry = approval_record.entry().to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest("Could not deserialize transition approval".into())))?;
        let registrant = get(input.machine_hash.clone(), GetOptions::default())?
            .ok_or(wasm_error!(WasmErrorInner::Guest("Machine root not found".into())))?;
        if approval.machine_hash != input.machine_hash
            || approval.authority_hash != input.authority_hash
            || approval.controller_agent != authority.controller_agent
            || approval.controller_agent != agent_info()?.agent_initial_pubkey
            || approval.predecessor_action != head_action
            || approval.new_status != input.new_status
            || approval.work_order_hash.as_ref() != input.work_order_hash.as_ref()
            || approval.valid_from < authority.valid_from
            || approval.valid_until > authority.valid_until
            || now < approval.valid_from
            || now > approval.valid_until
        {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "transition approval does not exactly authorize this machine update".into(),
            )));
        }
        if !verify_signature(
            registrant.action().author().clone(),
            approval.issuer_signature.clone(),
            approval.signed_payload(),
        )? {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "transition approval signature is invalid".into(),
            )));
        }
    }
    if now < authority.valid_from || now > authority.valid_until {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Controller authority is not currently valid".into(),
        )));
    }

    let authority_hash = input.authority_hash.clone();
    let previous_status = machine.status.clone();
    let updated = MachineEntry {
        status: input.new_status,
        current_work_order: input.work_order_hash.clone(),
        last_status_authority_hash: Some(authority_hash.clone()),
        last_status_transition_approval_hash: input.transition_approval_hash.clone(),
        ..machine
    };
    let update_hash = update_entry(head_action, EntryTypes::Machine(updated))?;

    let log = MachineStatusLog {
        machine_hash: input.machine_hash.clone(),
        machine_update_hash: update_hash.clone(),
        authority_hash,
        previous_status,
        new_status: input.new_status.clone(),
        work_order_hash: input.work_order_hash.clone(),
        transition_approval_hash: input.transition_approval_hash.clone(),
        changed_at: now,
    };
    let log_hash = create_entry(EntryTypes::StatusLog(log))?;
    create_link(
        input.machine_hash.clone(),
        log_hash.clone(),
        LinkTypes::MachineToStatusLog,
        (),
    )?;
    create_link(
        update_hash.clone(),
        log_hash,
        LinkTypes::MachineUpdateToStatusLog,
        (),
    )?;

    Ok(update_hash)

}

/// Get all machines that are currently Available.
#[hdk_extern]
pub fn get_available_machines(_: ()) -> ExternResult<Vec<Record>> {
    let all_path = Path::from("all_machines").typed(LinkTypes::AllMachines)?;
    let links = get_links(
        GetLinksInputBuilder::try_new(
            all_path.path_entry_hash()?,
            LinkTypes::AllMachines,
        )?
        .build(),
    )?;

    let mut available = Vec::new();
    for link in links {
        let Some(root_hash) = link.target.clone().into_action_hash() else {
            continue;
        };

        let MachineStateResolution::Resolved {
            status: MachineStatus::Available,
            head_action,
        } = get_current_machine_state(root_hash)?
        else {
            continue;
        };

        if let Some(record) = get(head_action, GetOptions::default())? {
            available.push(record);
        }
    }
    Ok(available)
}

/// Get machines by type via type anchor links.
#[hdk_extern]
pub fn get_machines_by_type(input: GetMachinesByTypeInput) -> ExternResult<Vec<Link>> {
    let tag = machine_type_tag(&input.machine_type);
    let type_path = Path::from(format!("machine_type/{tag}")).typed(LinkTypes::TypeToMachines)?;
    get_links(
        GetLinksInputBuilder::try_new(type_path.path_entry_hash()?, LinkTypes::TypeToMachines)?
            .build(),
    )
}

/// Produce a stable string tag for each machine type variant.
fn machine_type_tag(mt: &MachineType) -> String {
    match mt {
        MachineType::FDM => "fdm".to_string(),
        MachineType::SLA => "sla".to_string(),
        MachineType::SLS => "sls".to_string(),
        MachineType::CNC3Axis => "cnc3".to_string(),
        MachineType::CNC5Axis => "cnc5".to_string(),
        MachineType::LaserCutter => "laser".to_string(),
        MachineType::Lathe => "lathe".to_string(),
        MachineType::Assembly => "assembly".to_string(),
        MachineType::Custom(s) => format!("custom_{s}"),
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_input_serde() {
        let input = RegisterMachineInput {
            name: "CNC Mill #3".to_string(),
            machine_type: MachineType::CNC3Axis,
            capabilities: vec!["milling".to_string(), "drilling".to_string()],
            location: "Bay 2".to_string(),
            max_throughput_per_hour: 12,
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: RegisterMachineInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.name, "CNC Mill #3");
        assert_eq!(back.machine_type, MachineType::CNC3Axis);
    }

    #[test]
    fn test_machine_type_tag_variants() {
        assert_eq!(machine_type_tag(&MachineType::FDM), "fdm");
        assert_eq!(machine_type_tag(&MachineType::CNC5Axis), "cnc5");
        assert_eq!(machine_type_tag(&MachineType::LaserCutter), "laser");
        assert_eq!(
            machine_type_tag(&MachineType::Custom("Waterjet".to_string())),
            "custom_Waterjet"
        );
    }

    #[test]
    fn test_machine_state_resolution_serde() {
        let resolved = MachineStateResolution::Resolved {
            status: MachineStatus::Available,
            head_action: ActionHash::from_raw_36(vec![3; 36]),
        };
        let json = serde_json::to_string(&resolved).unwrap();
        let back: MachineStateResolution = serde_json::from_str(&json).unwrap();
        assert_eq!(back, resolved);

        let invalid = MachineStateResolution::InvalidRecord;
        let json = serde_json::to_string(&invalid).unwrap();
        let back: MachineStateResolution = serde_json::from_str(&json).unwrap();
        assert_eq!(back, invalid);

        let ambiguous = MachineStateResolution::Ambiguous {
            statuses: vec![MachineStatus::Available, MachineStatus::Running],
            head_actions: vec![
                ActionHash::from_raw_36(vec![4; 36]),
                ActionHash::from_raw_36(vec![5; 36]),
            ],
        };
        let json = serde_json::to_string(&ambiguous).unwrap();
        let back: MachineStateResolution = serde_json::from_str(&json).unwrap();
        assert_eq!(back, ambiguous);
    }

    #[test]
    fn test_grant_controller_input_serde() {
        let input = GrantMachineControllerInput {
            machine_hash: ActionHash::from_raw_36(vec![0; 36]),
            controller_agent: AgentPubKey::from_raw_32(vec![1; 32]),
            valid_from: Timestamp::from_micros(0),
            valid_until: Timestamp::from_micros(10),
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: GrantMachineControllerInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.machine_hash, input.machine_hash);
        assert_eq!(back.controller_agent, input.controller_agent);
    }

    #[test]
    fn test_update_status_input_serde() {
        let input = UpdateMachineStatusInput {
            machine_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            new_status: MachineStatus::Running,
            work_order_hash: Some(ActionHash::from_raw_36(vec![1u8; 36])),
            authority_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            transition_approval_hash: None,
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: UpdateMachineStatusInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.new_status, MachineStatus::Running);
        assert!(back.work_order_hash.is_some());
    }

    #[test]
    fn test_machine_status_all_variants_serde() {
        for s in [
            MachineStatus::Available,
            MachineStatus::Running,
            MachineStatus::Maintenance,
            MachineStatus::Offline,
        ] {
            let json = serde_json::to_string(&s).unwrap();
            let back: MachineStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(back, s);
        }
    }

    #[test]
    fn test_machine_entry_serde() {
        let entry = MachineEntry {
            name: "Lathe #1".to_string(),
            machine_type: MachineType::Lathe,
            capabilities: vec!["turning".to_string()],
            location: "Bay 3".to_string(),
            max_throughput_per_hour: 8,
            status: MachineStatus::Available,
            current_work_order: None,
            last_status_authority_hash: None,
            last_status_transition_approval_hash: None,
            registered_at: Timestamp::from_micros(0),
        };
        let json = serde_json::to_string(&entry).unwrap();
        assert!(json.contains("Lathe #1"));
        assert!(json.contains("\"status\":\"Available\""));
    }
}
