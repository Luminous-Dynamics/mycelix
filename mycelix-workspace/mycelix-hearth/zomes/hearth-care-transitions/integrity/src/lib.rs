// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Immutable Care transition evidence and profile-relative lifecycle admission.
//!
//! `CareCompletion` is revision-scoped completion evidence/attestation only.
//! It does not establish current assignment or admitted lifecycle completion.
//! `CareCompletionAdmissionV1` is a distinct immutable positive admission under
//! one closed profile and is validated independently of historical assignee
//! identity.

use hdi::prelude::*;
use hearth_authority_hdi::require_fresh_active_membership_from_chain;
use hearth_care_admission_proof::{
    AdmissionCandidate, CareCompletionAdmissionProfileV1 as StructuralProfile,
    CompletionEvidenceFact, FactId, MembershipDisposition, MembershipFact,
    ScheduleRootFact, evaluate_admission,
};
use hearth_care_integrity::CareSchedule;
use std::collections::HashSet;

const LEGACY_CARE_INTEGRITY_ZOME: &str = "hearth_care_integrity";
const LEGACY_CARE_SCHEDULE_ENTRY_INDEX: u8 = 0;
const TRANSITION_INTEGRITY_ZOME: &str = "hearth_care_transitions_integrity";
const CARE_COMPLETION_ENTRY_INDEX: u8 = 0;
const CARE_COMPLETION_ADMISSION_ENTRY_INDEX: u8 = 1;
const ADMISSION_SCHEMA_V1: u8 = 1;
const MAX_LEGACY_ANCESTRY_DEPTH: usize = 256;

/// Immutable revision-scoped evidence that `actor` attested completion concerning
/// one historical CareSchedule revision.
///
/// `assignee` means the assignee asserted by the referenced legacy revision. It
/// is not proof of current assignment. The signed Holochain Create action
/// timestamp is the canonical evidence timestamp.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CareCompletion {
    pub hearth_hash: ActionHash,
    pub schedule_hash: ActionHash,
    pub assignee: AgentPubKey,
    pub actor: AgentPubKey,
    pub actor_membership_hash: ActionHash,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum CareCompletionAdmissionProfileV1 {
    CreatorOrCurrentGuardianV1,
}

/// Immutable positive lifecycle admission under one declared profile.
///
/// Actor identity and admission time are carried by the signed Create action.
/// Historical CareSchedule assignee fields are never admission authority.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CareCompletionAdmissionV1 {
    pub schema_version: u8,
    pub profile: CareCompletionAdmissionProfileV1,
    pub hearth_hash: ActionHash,
    pub schedule_root_hash: ActionHash,
    pub evidence_hashes: Vec<ActionHash>,
    pub actor_membership_hash: ActionHash,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    /// Protocol entry index 0. Do not move.
    CareCompletion(CareCompletion),
    /// Protocol entry index 1. Append-only after published CareCompletion.
    CareCompletionAdmissionV1(CareCompletionAdmissionV1),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Protocol link index 0. Legacy CareSchedule revision -> completion evidence.
    ScheduleToCompletions,
    /// Protocol link index 1. Stable CareSchedule root -> completion admissions.
    ScheduleRootToCompletionAdmissions,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::CareCompletion(completion) => {
                    validate_care_completion(&completion, &action)
                }
                EntryTypes::CareCompletionAdmissionV1(admission) => {
                    validate_completion_admission(&admission, &action)
                }
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "Care transition evidence/admission is immutable and cannot be updated".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            tag,
            action,
        } => {
            if !tag.0.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Care transition index links must use an empty tag".into(),
                ));
            }
            match link_type {
                LinkTypes::ScheduleToCompletions => validate_schedule_completion_link(
                    base_address,
                    target_address,
                    &action.author,
                ),
                LinkTypes::ScheduleRootToCompletionAdmissions => {
                    validate_schedule_admission_link(
                        base_address,
                        target_address,
                        &action.author,
                    )
                }
            }
        }
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Care transition index links are append-only and cannot be deleted".into(),
        )),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Care transition evidence/admission is immutable and cannot be updated".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Care transition evidence/admission is immutable and cannot be deleted".into(),
        )),
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

fn validate_care_completion(
    completion: &CareCompletion,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if action.author != completion.actor {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion.actor must equal the Create action author".into(),
        ));
    }

    let schedule_record = must_get_valid_record(completion.schedule_hash.clone())?;
    let schedule_entry_def = match schedule_record.action().app_entry_def() {
        Some(entry_def) => entry_def,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "CareCompletion.schedule_hash must reference an application entry".into(),
            ));
        }
    };

    let dna = dna_info()?;
    if !is_legacy_care_schedule_entry_def(schedule_entry_def, &dna.zome_names) {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion.schedule_hash must reference hearth_care_integrity::CareSchedule"
                .into(),
        ));
    }

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
                "Referenced CareSchedule record is missing its entry".into(),
            ))
        })?;

    if schedule.hearth_hash != completion.hearth_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion Hearth does not match referenced CareSchedule".into(),
        ));
    }
    if schedule.assigned_to != completion.assignee {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion assignee does not match referenced CareSchedule revision".into(),
        ));
    }

    // This proves authority to author evidence only. It does not prove current
    // assignment or lifecycle admission under Amendment 002 / #2091.
    let role = match require_fresh_active_membership_from_chain(
        action,
        &completion.actor,
        &completion.hearth_hash,
        &completion.actor_membership_hash,
    )? {
        Ok(role) => role,
        Err(error) => {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "CareCompletion membership authority is invalid: {error}"
            )));
        }
    };

    if completion.actor != completion.assignee && !role.is_guardian() {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the revision-scoped assignee or a current guardian may author CareCompletion evidence"
                .into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

struct LegacyRootResolution {
    root_hash: ActionHash,
    hearth_hash: ActionHash,
    creator: AgentPubKey,
}

fn resolve_legacy_schedule_root(
    revision_hash: &ActionHash,
) -> ExternResult<Result<LegacyRootResolution, String>> {
    let dna = dna_info()?;
    let mut current_hash = revision_hash.clone();
    let mut expected_hearth: Option<ActionHash> = None;
    let mut seen = HashSet::new();

    for _ in 0..MAX_LEGACY_ANCESTRY_DEPTH {
        let raw = current_hash.get_raw_39().to_vec();
        if !seen.insert(raw) {
            return Ok(Err(
                "CareSchedule update ancestry contains a cycle".to_string(),
            ));
        }

        let record = must_get_valid_record(current_hash.clone())?;
        let entry_def = match record.action().app_entry_def() {
            Some(entry_def) => entry_def,
            None => {
                return Ok(Err(
                    "CareSchedule ancestry reached a non-application action".to_string(),
                ));
            }
        };
        if !is_legacy_care_schedule_entry_def(entry_def, &dna.zome_names) {
            return Ok(Err(
                "CareSchedule ancestry reached a non-canonical legacy CareSchedule entry"
                    .to_string(),
            ));
        }

        let schedule: CareSchedule = record
            .entry()
            .to_app_option()
            .map_err(|error| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode CareSchedule ancestry record: {error}"
                )))
            })?
            .ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "CareSchedule ancestry record is missing its entry".into(),
                ))
            })?;

        match &expected_hearth {
            Some(hearth) if hearth != &schedule.hearth_hash => {
                return Ok(Err(
                    "CareSchedule update ancestry changes Hearth binding".to_string(),
                ));
            }
            None => expected_hearth = Some(schedule.hearth_hash.clone()),
            _ => {}
        }

        match record.action() {
            Action::Create(_) => {
                return Ok(Ok(LegacyRootResolution {
                    root_hash: current_hash,
                    hearth_hash: schedule.hearth_hash,
                    creator: record.action().author().clone(),
                }));
            }
            Action::Update(update) => {
                current_hash = update.original_action_address.clone();
            }
            _ => {
                return Ok(Err(
                    "CareSchedule ancestry must contain only Create/Update actions".to_string(),
                ));
            }
        }
    }

    Ok(Err(format!(
        "CareSchedule ancestry exceeds maximum depth {MAX_LEGACY_ANCESTRY_DEPTH}"
    )))
}

struct ResolvedCompletionEvidence {
    evidence_hash: ActionHash,
    completion: CareCompletion,
    root: LegacyRootResolution,
}

fn resolve_completion_evidence(
    evidence_hash: &ActionHash,
) -> ExternResult<Result<ResolvedCompletionEvidence, String>> {
    let record = must_get_valid_record(evidence_hash.clone())?;
    let entry_def = match record.action().app_entry_def() {
        Some(entry_def) => entry_def,
        None => {
            return Ok(Err(
                "Completion evidence hash must reference an application entry".to_string(),
            ));
        }
    };
    let dna = dna_info()?;
    if !is_transition_entry_def(
        entry_def,
        &dna.zome_names,
        CARE_COMPLETION_ENTRY_INDEX,
    ) {
        return Ok(Err(
            "Admission evidence must reference canonical CareCompletion entry type".to_string(),
        ));
    }

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
                "CareCompletion evidence record is missing its entry".into(),
            ))
        })?;

    let root = match resolve_legacy_schedule_root(&completion.schedule_hash)? {
        Ok(root) => root,
        Err(message) => return Ok(Err(message)),
    };

    Ok(Ok(ResolvedCompletionEvidence {
        evidence_hash: evidence_hash.clone(),
        completion,
        root,
    }))
}

fn validate_completion_admission(
    admission: &CareCompletionAdmissionV1,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if admission.schema_version != ADMISSION_SCHEMA_V1 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Unsupported CareCompletionAdmission schema version {}",
            admission.schema_version
        )));
    }

    let structural_profile = match admission.profile {
        CareCompletionAdmissionProfileV1::CreatorOrCurrentGuardianV1 => {
            StructuralProfile::CreatorOrCurrentGuardianV1
        }
    };

    if admission.evidence_hashes.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletionAdmission requires at least one completion-evidence hash".into(),
        ));
    }
    if admission.evidence_hashes.len() > 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletionAdmission supports at most 32 completion-evidence hashes".into(),
        ));
    }
    for pair in admission.evidence_hashes.windows(2) {
        match pair[0].get_raw_39().cmp(pair[1].get_raw_39()) {
            std::cmp::Ordering::Less => {}
            std::cmp::Ordering::Equal => {
                return Ok(ValidateCallbackResult::Invalid(
                    "CareCompletionAdmission evidence hashes must be unique".into(),
                ));
            }
            std::cmp::Ordering::Greater => {
                return Ok(ValidateCallbackResult::Invalid(
                    "CareCompletionAdmission evidence hashes must be strictly sorted by raw ActionHash bytes"
                        .into(),
                ));
            }
        }
    }

    let root = match resolve_legacy_schedule_root(&admission.schedule_root_hash)? {
        Ok(root) => root,
        Err(message) => return Ok(ValidateCallbackResult::Invalid(message)),
    };
    if root.root_hash != admission.schedule_root_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletionAdmission.schedule_root_hash must be the original CareSchedule Create action"
                .into(),
        ));
    }
    if root.hearth_hash != admission.hearth_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletionAdmission Hearth does not match the canonical schedule root".into(),
        ));
    }

    let role = match require_fresh_active_membership_from_chain(
        action,
        &action.author,
        &admission.hearth_hash,
        &admission.actor_membership_hash,
    )? {
        Ok(role) => role,
        Err(error) => {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "CareCompletionAdmission membership authority is invalid: {error}"
            )));
        }
    };

    let mut evidence_facts = Vec::with_capacity(admission.evidence_hashes.len());
    for evidence_hash in &admission.evidence_hashes {
        let evidence = match resolve_completion_evidence(evidence_hash)? {
            Ok(evidence) => evidence,
            Err(message) => return Ok(ValidateCallbackResult::Invalid(message)),
        };

        if evidence.root.root_hash != admission.schedule_root_hash {
            return Ok(ValidateCallbackResult::Invalid(
                "CareCompletionAdmission evidence normalizes to a different schedule root".into(),
            ));
        }
        if evidence.root.hearth_hash != admission.hearth_hash
            || evidence.completion.hearth_hash != admission.hearth_hash
        {
            return Ok(ValidateCallbackResult::Invalid(
                "CareCompletionAdmission evidence belongs to a different Hearth".into(),
            ));
        }

        evidence_facts.push(CompletionEvidenceFact {
            id: FactId::new(evidence.evidence_hash.get_raw_39().to_vec()),
            normalized_root_id: FactId::new(evidence.root.root_hash.get_raw_39().to_vec()),
            hearth_id: FactId::new(admission.hearth_hash.get_raw_39().to_vec()),
            attestor_actor_id: FactId::new(evidence.completion.actor.get_raw_39().to_vec()),
            revision_assignee_actor_id: FactId::new(
                evidence.completion.assignee.get_raw_39().to_vec(),
            ),
        });
    }

    let structural_candidate = AdmissionCandidate {
        schema_version: admission.schema_version,
        profile: structural_profile,
        actor_id: FactId::new(action.author.get_raw_39().to_vec()),
        root: ScheduleRootFact {
            root_id: FactId::new(root.root_hash.get_raw_39().to_vec()),
            hearth_id: FactId::new(root.hearth_hash.get_raw_39().to_vec()),
            creator_actor_id: FactId::new(root.creator.get_raw_39().to_vec()),
        },
        evidence: evidence_facts,
        membership: MembershipFact {
            proof_id: FactId::new(admission.actor_membership_hash.get_raw_39().to_vec()),
            actor_id: FactId::new(action.author.get_raw_39().to_vec()),
            hearth_id: FactId::new(admission.hearth_hash.get_raw_39().to_vec()),
            disposition: MembershipDisposition::FreshActive {
                is_guardian: role.is_guardian(),
            },
        },
    };

    if let Err(error) = evaluate_admission(structural_candidate) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "CareCompletionAdmission structural semantics rejected: {error:?}"
        )));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_schedule_completion_link(
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    link_author: &AgentPubKey,
) -> ExternResult<ValidateCallbackResult> {
    let schedule_hash = match ActionHash::try_from(base_address) {
        Ok(hash) => hash,
        Err(_) => {
            return Ok(ValidateCallbackResult::Invalid(
                "ScheduleToCompletions base must be a CareSchedule ActionHash".into(),
            ));
        }
    };
    let completion_hash = match ActionHash::try_from(target_address) {
        Ok(hash) => hash,
        Err(_) => {
            return Ok(ValidateCallbackResult::Invalid(
                "ScheduleToCompletions target must be a CareCompletion ActionHash".into(),
            ));
        }
    };

    let completion_record = must_get_valid_record(completion_hash)?;
    if completion_record.action().author() != link_author {
        return Ok(ValidateCallbackResult::Invalid(
            "ScheduleToCompletions link must be authored by the CareCompletion actor".into(),
        ));
    }

    let completion: CareCompletion = completion_record
        .entry()
        .to_app_option()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode CareCompletion link target: {error}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "CareCompletion link target is missing its entry".into(),
            ))
        })?;

    if completion.actor != *link_author {
        return Ok(ValidateCallbackResult::Invalid(
            "ScheduleToCompletions link author does not match CareCompletion.actor".into(),
        ));
    }
    if completion.schedule_hash != schedule_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "ScheduleToCompletions target references a different CareSchedule revision".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_schedule_admission_link(
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    link_author: &AgentPubKey,
) -> ExternResult<ValidateCallbackResult> {
    let root_hash = match ActionHash::try_from(base_address) {
        Ok(hash) => hash,
        Err(_) => {
            return Ok(ValidateCallbackResult::Invalid(
                "ScheduleRootToCompletionAdmissions base must be a CareSchedule root ActionHash"
                    .into(),
            ));
        }
    };
    let admission_hash = match ActionHash::try_from(target_address) {
        Ok(hash) => hash,
        Err(_) => {
            return Ok(ValidateCallbackResult::Invalid(
                "ScheduleRootToCompletionAdmissions target must be a CareCompletionAdmission ActionHash"
                    .into(),
            ));
        }
    };

    let root = match resolve_legacy_schedule_root(&root_hash)? {
        Ok(root) => root,
        Err(message) => return Ok(ValidateCallbackResult::Invalid(message)),
    };
    if root.root_hash != root_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "ScheduleRootToCompletionAdmissions base must be the original CareSchedule Create action"
                .into(),
        ));
    }

    let record = must_get_valid_record(admission_hash)?;
    if record.action().author() != link_author {
        return Ok(ValidateCallbackResult::Invalid(
            "ScheduleRootToCompletionAdmissions link must be authored by the admission actor"
                .into(),
        ));
    }

    let entry_def = match record.action().app_entry_def() {
        Some(entry_def) => entry_def,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "Admission link target must reference an application entry".into(),
            ));
        }
    };
    let dna = dna_info()?;
    if !is_transition_entry_def(
        entry_def,
        &dna.zome_names,
        CARE_COMPLETION_ADMISSION_ENTRY_INDEX,
    ) {
        return Ok(ValidateCallbackResult::Invalid(
            "Admission link target must reference canonical CareCompletionAdmissionV1"
                .into(),
        ));
    }

    let admission: CareCompletionAdmissionV1 = record
        .entry()
        .to_app_option()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode CareCompletionAdmission link target: {error}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "CareCompletionAdmission link target is missing its entry".into(),
            ))
        })?;

    if admission.schedule_root_hash != root_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Admission link target is bound to a different CareSchedule root".into(),
        ));
    }
    if admission.hearth_hash != root.hearth_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Admission link target Hearth does not match the CareSchedule root".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Bind a referenced AppEntryDef to the historical legacy CareSchedule type.
fn is_legacy_care_schedule_entry_def(
    entry_def: &AppEntryDef,
    integrity_zome_names: &[ZomeName],
) -> bool {
    let zome_name = integrity_zome_names.get(entry_def.zome_index.0 as usize);
    zome_name == Some(&ZomeName::new(LEGACY_CARE_INTEGRITY_ZOME))
        && entry_def.entry_index.0 == LEGACY_CARE_SCHEDULE_ENTRY_INDEX
}

fn is_transition_entry_def(
    entry_def: &AppEntryDef,
    integrity_zome_names: &[ZomeName],
    expected_entry_index: u8,
) -> bool {
    let zome_name = integrity_zome_names.get(entry_def.zome_index.0 as usize);
    zome_name == Some(&ZomeName::new(TRANSITION_INTEGRITY_ZOME))
        && entry_def.entry_index.0 == expected_entry_index
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_types::MemberRole;

    fn app_entry_def(zome_index: u8, entry_index: u8) -> AppEntryDef {
        AppEntryDef {
            zome_index: zome_index.into(),
            entry_index: entry_index.into(),
            visibility: EntryVisibility::Public,
        }
    }

    fn zomes() -> Vec<ZomeName> {
        vec![
            ZomeName::new("hearth_kinship_integrity"),
            ZomeName::new(LEGACY_CARE_INTEGRITY_ZOME),
            ZomeName::new(TRANSITION_INTEGRITY_ZOME),
        ]
    }

    #[test]
    fn legacy_schedule_type_provenance_accepts_exact_zome_and_entry() {
        assert!(is_legacy_care_schedule_entry_def(
            &app_entry_def(1, LEGACY_CARE_SCHEDULE_ENTRY_INDEX),
            &zomes(),
        ));
    }

    #[test]
    fn legacy_schedule_type_provenance_rejects_same_entry_index_from_other_zome() {
        assert!(!is_legacy_care_schedule_entry_def(
            &app_entry_def(0, LEGACY_CARE_SCHEDULE_ENTRY_INDEX),
            &zomes(),
        ));
    }

    #[test]
    fn transition_type_provenance_preserves_published_indexes() {
        assert!(is_transition_entry_def(
            &app_entry_def(2, CARE_COMPLETION_ENTRY_INDEX),
            &zomes(),
            CARE_COMPLETION_ENTRY_INDEX,
        ));
        assert!(is_transition_entry_def(
            &app_entry_def(2, CARE_COMPLETION_ADMISSION_ENTRY_INDEX),
            &zomes(),
            CARE_COMPLETION_ADMISSION_ENTRY_INDEX,
        ));
        assert!(!is_transition_entry_def(
            &app_entry_def(2, CARE_COMPLETION_ENTRY_INDEX),
            &zomes(),
            CARE_COMPLETION_ADMISSION_ENTRY_INDEX,
        ));
    }

    #[test]
    fn transition_type_provenance_rejects_other_zome_lookalikes() {
        assert!(!is_transition_entry_def(
            &app_entry_def(0, CARE_COMPLETION_ADMISSION_ENTRY_INDEX),
            &zomes(),
            CARE_COMPLETION_ADMISSION_ENTRY_INDEX,
        ));
    }

    #[test]
    fn guardian_policy_matches_current_hearth_roles() {
        assert!(MemberRole::Founder.is_guardian());
        assert!(MemberRole::Elder.is_guardian());
        assert!(MemberRole::Adult.is_guardian());
        assert!(!MemberRole::Youth.is_guardian());
        assert!(!MemberRole::Child.is_guardian());
        assert!(!MemberRole::Guest.is_guardian());
        assert!(!MemberRole::Ancestor.is_guardian());
    }
}
