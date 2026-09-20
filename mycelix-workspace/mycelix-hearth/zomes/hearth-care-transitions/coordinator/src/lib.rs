// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Coordinator for immutable Care lifecycle transition evidence and admission.
//!
//! Coordinator derivation is an early-failure/construction boundary only.
//! `hearth_care_transitions_integrity` independently proves every authority
//! statement before completion evidence or admission becomes valid DHT state.

use hdk::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_care_transitions_integrity::{
    CareCompletion, CareCompletionAdmissionProfileV1, CareCompletionAdmissionV1, EntryTypes,
    LinkTypes,
};
use hearth_coordinator_common::decode_zome_response;
use hearth_kinship_integrity::HearthMembership;
use hearth_types::MembershipStatus;
use mycelix_bridge_common::civic_requirement_basic;

const ADMISSION_SCHEMA_V1: u8 = 1;
const MAX_ADMISSION_EVIDENCE: usize = 32;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CompleteTaskV2Input {
    pub schedule_hash: ActionHash,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AdmitCompletionV1Input {
    pub schedule_root_hash: ActionHash,
    pub evidence_hashes: Vec<ActionHash>,
    pub profile: CareCompletionAdmissionProfileV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct MembershipCandidate {
    action_hash: ActionHash,
    action_seq: u32,
    status: MembershipStatus,
}

/// Create immutable actor-authored completion evidence.
///
/// The caller does not supply actor identity, Hearth, assignee, or membership
/// proof. Those values are derived from signed/local state and canonical
/// Kinship records. Integrity independently re-proves every authority claim.
#[hdk_extern]
pub fn complete_task_v2(input: CompleteTaskV2Input) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "complete_task_v2",
    )?;

    let actor = agent_info()?.agent_initial_pubkey;
    let schedule_record = get(input.schedule_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Referenced CareSchedule was not found".into()
        ))
    })?;
    let schedule: CareSchedule = schedule_record
        .entry()
        .to_app_option()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode referenced CareSchedule: {error}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Referenced CareSchedule record is missing its entry".into()
            ))
        })?;

    // Coordinator-side type decoding is only fast feedback. Transition
    // integrity independently proves the referenced action's actual zome and
    // entry index before accepting the CareCompletion.
    let actor_membership_hash = latest_active_membership_hash(
        &schedule.hearth_hash,
        &actor,
    )?;

    let completion = CareCompletion {
        hearth_hash: schedule.hearth_hash,
        schedule_hash: input.schedule_hash.clone(),
        assignee: schedule.assigned_to,
        actor,
        actor_membership_hash,
    };

    let completion_hash = create_entry(&EntryTypes::CareCompletion(completion))?;
    create_link(
        input.schedule_hash,
        completion_hash.clone(),
        LinkTypes::ScheduleToCompletions,
        (),
    )?;

    get(completion_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve created CareCompletion".into()
        ))
    })
}

/// Create one immutable positive lifecycle admission under an explicit v1
/// profile.
///
/// The caller supplies references and the profile selection only. Actor,
/// Hearth, and membership evidence are derived. This coordinator check is not
/// authority: transition integrity independently normalizes the stable root,
/// validates every evidence reference, re-proves fresh membership, and proves
/// root-creator/current-guardian authority.
#[hdk_extern]
pub fn admit_completion_v1(input: AdmitCompletionV1Input) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "admit_completion_v1",
    )?;

    let actor = agent_info()?.agent_initial_pubkey;
    let root_record = get(
        input.schedule_root_hash.clone(),
        GetOptions::default(),
    )?
    .ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Referenced CareSchedule root was not found".into()
        ))
    })?;

    // Early feedback only. Integrity independently proves canonical legacy Care
    // type provenance and that this exact hash is the original Create/root.
    if !matches!(root_record.action(), Action::Create(_)) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "schedule_root_hash must reference a CareSchedule Create action".into()
        )));
    }
    let root_schedule: CareSchedule = root_record
        .entry()
        .to_app_option()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode referenced CareSchedule root: {error}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Referenced CareSchedule root is missing its entry".into()
            ))
        })?;

    let evidence_hashes = canonicalize_admission_evidence(input.evidence_hashes)
        .map_err(|message| wasm_error!(WasmErrorInner::Guest(message.into())))?;
    let actor_membership_hash = latest_active_membership_hash(
        &root_schedule.hearth_hash,
        &actor,
    )?;

    let admission = CareCompletionAdmissionV1 {
        schema_version: ADMISSION_SCHEMA_V1,
        profile: input.profile,
        hearth_hash: root_schedule.hearth_hash,
        schedule_root_hash: input.schedule_root_hash.clone(),
        evidence_hashes,
        actor_membership_hash,
    };

    let admission_hash = create_entry(&EntryTypes::CareCompletionAdmissionV1(admission))?;
    create_link(
        input.schedule_root_hash,
        admission_hash.clone(),
        LinkTypes::ScheduleRootToCompletionAdmissions,
        (),
    )?;

    get(admission_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve created CareCompletionAdmissionV1".into()
        ))
    })
}

/// Return raw immutable completion evidence for a CareSchedule revision.
///
/// This intentionally does not claim a complete lifecycle projection while
/// legacy CareSchedule status remains mutable. #2016 owns legacy/v2
/// reconciliation before broad client rollout.
#[hdk_extern]
pub fn get_schedule_completion_evidence(
    schedule_hash: ActionHash,
) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            schedule_hash.clone(),
            LinkTypes::ScheduleToCompletions,
        )?,
        GetStrategy::default(),
    )?;

    let mut evidence = Vec::with_capacity(links.len());
    for link in links {
        let completion_hash = ActionHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(
                "Invalid ScheduleToCompletions link target".into()
            ))
        })?;
        let record = get(completion_hash, GetOptions::default())?.ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "CareCompletion link target was not found".into()
            ))
        })?;
        let completion: CareCompletion = record
            .entry()
            .to_app_option()
            .map_err(|error| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode CareCompletion evidence: {error}"
                )))
            })?
            .ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "CareCompletion evidence record is missing its entry".into()
                ))
            })?;

        if completion.schedule_hash != schedule_hash {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "CareCompletion evidence is bound to a different CareSchedule".into()
            )));
        }
        evidence.push(record);
    }

    sort_records_canonically(&mut evidence);
    Ok(evidence)
}

/// Return raw immutable admission records indexed by the stable CareSchedule
/// root. A returned record is admission evidence under its declared profile;
/// this query does not compute lifecycle agreement/disagreement with legacy
/// state and does not imply universal completion or obligation discharge.
#[hdk_extern]
pub fn get_schedule_completion_admissions(
    schedule_root_hash: ActionHash,
) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            schedule_root_hash.clone(),
            LinkTypes::ScheduleRootToCompletionAdmissions,
        )?,
        GetStrategy::default(),
    )?;

    let mut admissions = Vec::with_capacity(links.len());
    for link in links {
        let admission_hash = ActionHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(
                "Invalid ScheduleRootToCompletionAdmissions link target".into()
            ))
        })?;
        let record = get(admission_hash, GetOptions::default())?.ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "CareCompletionAdmission link target was not found".into()
            ))
        })?;
        let admission: CareCompletionAdmissionV1 = record
            .entry()
            .to_app_option()
            .map_err(|error| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode CareCompletionAdmissionV1: {error}"
                )))
            })?
            .ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "CareCompletionAdmission record is missing its entry".into()
                ))
            })?;

        if admission.schedule_root_hash != schedule_root_hash {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "CareCompletionAdmission is indexed under a different schedule root".into()
            )));
        }
        admissions.push(record);
    }

    sort_records_canonically(&mut admissions);
    Ok(admissions)
}

fn sort_records_canonically(records: &mut [Record]) {
    records.sort_by(|left, right| {
        left.action()
            .timestamp()
            .cmp(right.action().timestamp())
            .then_with(|| left.action_address().cmp(right.action_address()))
    });
}

fn canonicalize_admission_evidence(
    mut evidence_hashes: Vec<ActionHash>,
) -> Result<Vec<ActionHash>, &'static str> {
    if evidence_hashes.is_empty() {
        return Err("Completion admission requires at least one evidence hash");
    }
    if evidence_hashes.len() > MAX_ADMISSION_EVIDENCE {
        return Err("Completion admission supports at most 32 evidence hashes");
    }

    evidence_hashes.sort_by(|left, right| left.get_raw_39().cmp(right.get_raw_39()));
    if evidence_hashes
        .windows(2)
        .any(|pair| pair[0] == pair[1])
    {
        return Err("Completion admission evidence hashes must be unique");
    }
    Ok(evidence_hashes)
}

/// Resolve the caller's latest known Kinship membership revision for a Hearth
/// and require it to be Active before attempting to author evidence/admission.
///
/// This is a UX/early-failure guard, not the authority boundary. The transition
/// integrity zome replays deterministic source-chain evidence and rejects stale
/// proof hashes even if this coordinator observed incomplete DHT state.
fn latest_active_membership_hash(
    hearth_hash: &ActionHash,
    actor: &AgentPubKey,
) -> ExternResult<ActionHash> {
    let response = call(
        CallTargetCell::Local,
        ZomeName::new("hearth_kinship"),
        FunctionName::new("get_hearth_members"),
        None,
        hearth_hash.clone(),
    )?;
    let records: Vec<Record> = decode_zome_response(response, "get_hearth_members")?;

    let mut candidates = Vec::new();
    for record in records {
        let membership: HearthMembership = record
            .entry()
            .to_app_option()
            .map_err(|error| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode HearthMembership while preparing Care transition: {error}"
                )))
            })?
            .ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "Hearth membership record is missing its entry".into()
                ))
            })?;

        if membership.hearth_hash != *hearth_hash || membership.agent != *actor {
            continue;
        }
        if record.action().author() != actor {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Kinship returned a membership whose author does not match its actor".into()
            )));
        }

        candidates.push(MembershipCandidate {
            action_hash: record.action_address().clone(),
            action_seq: record.action().action_seq(),
            status: membership.status,
        });
    }

    select_latest_active_membership(&candidates)
        .map_err(|message| wasm_error!(WasmErrorInner::Guest(message.into())))
}

/// Pure fail-closed selector used only as coordinator-side early feedback.
fn select_latest_active_membership(
    candidates: &[MembershipCandidate],
) -> Result<ActionHash, &'static str> {
    let max_seq = candidates
        .iter()
        .map(|candidate| candidate.action_seq)
        .max()
        .ok_or("Caller has no Hearth membership record")?;

    let mut latest = candidates
        .iter()
        .filter(|candidate| candidate.action_seq == max_seq);
    let selected = latest
        .next()
        .ok_or("Caller has no Hearth membership record")?;
    if latest.next().is_some() {
        return Err("Caller membership history is ambiguous at the latest source-chain sequence");
    }
    if selected.status != MembershipStatus::Active {
        return Err("Caller's latest Hearth membership is not Active");
    }

    Ok(selected.action_hash.clone())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hash(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn candidate(byte: u8, seq: u32, status: MembershipStatus) -> MembershipCandidate {
        MembershipCandidate {
            action_hash: hash(byte),
            action_seq: seq,
            status,
        }
    }

    #[test]
    fn latest_active_membership_is_selected() {
        let older = candidate(0x11, 3, MembershipStatus::Active);
        let latest = candidate(0x12, 7, MembershipStatus::Active);
        assert_eq!(
            select_latest_active_membership(&[older, latest.clone()]),
            Ok(latest.action_hash)
        );
    }

    #[test]
    fn stale_active_record_cannot_hide_latest_departure() {
        let active = candidate(0x11, 3, MembershipStatus::Active);
        let departed = candidate(0x12, 7, MembershipStatus::Departed);
        assert_eq!(
            select_latest_active_membership(&[active, departed]),
            Err("Caller's latest Hearth membership is not Active")
        );
    }

    #[test]
    fn ambiguous_latest_membership_fails_closed() {
        let left = candidate(0x11, 7, MembershipStatus::Active);
        let right = candidate(0x12, 7, MembershipStatus::Active);
        assert_eq!(
            select_latest_active_membership(&[left, right]),
            Err("Caller membership history is ambiguous at the latest source-chain sequence")
        );
    }

    #[test]
    fn missing_membership_fails_closed() {
        assert_eq!(
            select_latest_active_membership(&[]),
            Err("Caller has no Hearth membership record")
        );
    }

    #[test]
    fn admission_evidence_is_sorted_canonically() {
        assert_eq!(
            canonicalize_admission_evidence(vec![hash(0x33), hash(0x11), hash(0x22)]),
            Ok(vec![hash(0x11), hash(0x22), hash(0x33)])
        );
    }

    #[test]
    fn admission_evidence_rejects_duplicates_and_empty_sets() {
        assert_eq!(
            canonicalize_admission_evidence(vec![]),
            Err("Completion admission requires at least one evidence hash")
        );
        assert_eq!(
            canonicalize_admission_evidence(vec![hash(0x11), hash(0x11)]),
            Err("Completion admission evidence hashes must be unique")
        );
    }

    #[test]
    fn admission_evidence_rejects_more_than_32_references() {
        let hashes = (0u8..33).map(hash).collect();
        assert_eq!(
            canonicalize_admission_evidence(hashes),
            Err("Completion admission supports at most 32 evidence hashes")
        );
    }
}
