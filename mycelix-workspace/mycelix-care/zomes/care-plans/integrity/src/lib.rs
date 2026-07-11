// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Care Plans Integrity Zome
//! Defines entry types and validation for coordinated care plans and sessions.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Type of care being provided
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum CareType {
    Childcare,
    Eldercare,
    DisabilitySupport,
    PostSurgery,
    MentalHealth,
    Respite,
    Other(String),
}

impl CareType {
    pub fn anchor_key(&self) -> String {
        match self {
            CareType::Childcare => "childcare".to_string(),
            CareType::Eldercare => "eldercare".to_string(),
            CareType::DisabilitySupport => "disability".to_string(),
            CareType::PostSurgery => "postsurgery".to_string(),
            CareType::MentalHealth => "mentalhealth".to_string(),
            CareType::Respite => "respite".to_string(),
            CareType::Other(s) => format!("other_{}", s.to_lowercase().replace(' ', "_")),
        }
    }
}

/// Status of a care plan
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum PlanStatus {
    Draft,
    Active,
    Paused,
    Completed,
    Cancelled,
}

/// A coordinated care plan for ongoing care needs
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CarePlan {
    /// The person receiving care
    pub recipient: AgentPubKey,
    /// Title of the care plan
    pub title: String,
    /// Detailed description of care needs
    pub description: String,
    /// Type of care
    pub care_type: CareType,
    /// Schedule description (e.g. "Mon/Wed/Fri mornings")
    pub schedule: String,
    /// List of assigned caregivers
    pub caregivers: Vec<AgentPubKey>,
    /// Current plan status
    pub status: PlanStatus,
    /// When the plan was created
    pub created_at: Timestamp,
    /// When the plan was last updated
    pub updated_at: Timestamp,
    /// Estimated total hours per week
    pub hours_per_week: f32,
    /// Special instructions or notes
    pub special_instructions: String,
}

/// A logged care session within a plan
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CareSession {
    /// The care plan this session belongs to
    pub plan_hash: ActionHash,
    /// The caregiver who provided care
    pub caregiver: AgentPubKey,
    /// When the session started
    pub started_at: Timestamp,
    /// When the session ended
    pub ended_at: Timestamp,
    /// Duration in hours
    pub hours: f32,
    /// Session notes
    pub notes: String,
    /// Tasks that were completed during the session
    pub tasks_completed: Vec<String>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    CarePlan(CarePlan),
    CareSession(CareSession),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// All care plans anchor
    AllPlans,
    /// Care type to plans
    TypeToPlans,
    /// Recipient to their care plans
    RecipientToPlans,
    /// Caregiver to plans they are assigned to
    CaregiverToPlans,
    /// Plan to its sessions
    PlanToSessions,
    /// Caregiver to their sessions
    CaregiverToSessions,
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
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::CarePlan(plan) => validate_create_plan(action, plan),
                EntryTypes::CareSession(session) => validate_create_session(action, session),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash: _,
                original_entry_hash: _,
            } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action,
            base_address: _,
            target_address: _,
            tag: _,
            action,
        } => {
            // Previously accepted unconditionally regardless of author --
            // the coordinator never calls delete_link here, so this is
            // pure hardening (zero functional impact). Found + fixed
            // 2026-07-09 during the P0 author-binding pass.
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original link creator can delete a link".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 5th confirmed instance of this exact
            // bug pattern this pass. Found + fixed 2026-07-09 during the P0
            // author-binding pass. Route through the same per-type
            // validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Also previously fully permissive. The coordinator never
            // calls delete_entry here, so this is pure hardening, zero
            // functional impact.
            let original = must_get_action(action.deletes_address.clone())?;
            if action.author != *original.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete an entry".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// Shared per-entry-type update validation, called from BOTH the
/// StoreEntry (OpEntry::UpdateEntry) and RegisterUpdate DHT-op
/// perspectives so they agree.
fn validate_update_entry_type(
    action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::CarePlan(plan) => validate_update_plan(action, plan),
        EntryTypes::CareSession(_) => Ok(ValidateCallbackResult::Invalid(
            "Care sessions are immutable".into(),
        )),
    }
}

fn validate_create_plan(action: Create, plan: CarePlan) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: CarePlan has no dedicated "creator" field, but the
    // coordinator's update_plan_status already restricts control to the
    // recipient or an assigned caregiver -- creation must follow the same
    // invariant, otherwise any agent could create a plan naming an
    // arbitrary victim as recipient and themselves as caregiver to gain
    // standing over it. Found + fixed 2026-07-09 during the P0
    // author-binding pass.
    if action.author != plan.recipient && !plan.caregivers.contains(&action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "Committing agent must be the recipient or an assigned caregiver".into(),
        ));
    }

    if plan.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Plan title cannot be empty".into(),
        ));
    }
    if plan.title.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Plan title must be 256 characters or fewer".into(),
        ));
    }
    if plan.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Plan description cannot be empty".into(),
        ));
    }
    if plan.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Plan description must be 4096 characters or fewer".into(),
        ));
    }
    if plan.schedule.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Schedule cannot be empty".into(),
        ));
    }
    if plan.schedule.len() > 512 {
        return Ok(ValidateCallbackResult::Invalid(
            "Schedule must be 512 characters or fewer".into(),
        ));
    }
    if plan.caregivers.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "At least one caregiver must be assigned".into(),
        ));
    }
    if plan.caregivers.len() > 50 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot have more than 50 caregivers".into(),
        ));
    }
    if plan.hours_per_week <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours per week must be positive".into(),
        ));
    }
    if plan.hours_per_week > 168.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours per week cannot exceed 168".into(),
        ));
    }
    if plan.special_instructions.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Special instructions must be 4096 characters or fewer".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a CarePlan update (the coordinator's update_plan_status only
/// ever changes status/updated_at).
///
/// Previously accepted ANY field change from ANY agent unconditionally.
/// Hardened 2026-07-09 during the P0 author-binding pass: the committing
/// agent must be the recipient or an assigned caregiver (mirroring
/// update_plan_status's own check, which a modified coordinator could
/// otherwise bypass), and only status/updated_at may actually change --
/// recipient/caregivers/title/description/etc. are immutable here (there
/// is no coordinator flow for editing plan details after creation).
fn validate_update_plan(action: Update, plan: CarePlan) -> ExternResult<ValidateCallbackResult> {
    if plan.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Plan title cannot be empty".into(),
        ));
    }
    if plan.caregivers.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "At least one caregiver must be assigned".into(),
        ));
    }

    if action.author != plan.recipient && !plan.caregivers.contains(&action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "Committing agent must be the recipient or an assigned caregiver".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: CarePlan = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original care plan not found".into()
        )))?;

    if plan.recipient != original.recipient
        || plan.title != original.title
        || plan.description != original.description
        || plan.care_type != original.care_type
        || plan.schedule != original.schedule
        || plan.caregivers != original.caregivers
        || plan.created_at != original.created_at
        || plan.hours_per_week != original.hours_per_week
        || plan.special_instructions != original.special_instructions
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/updated_at can change on a care plan update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_session(
    action: Create,
    session: CareSession,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's log_session now derives `caregiver`
    // from agent_info() rather than trusting caller input, but that's
    // bypassable by a modified coordinator -- the integrity validator is
    // the real security boundary. Without this, any agent could log a
    // session attributing hours to a DIFFERENT, real caregiver (the
    // existing check only verified the claimed caregiver was assigned to
    // the plan, not that the caller IS that caregiver). Found + fixed
    // 2026-07-09 during the P0 author-binding pass.
    if session.caregiver != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Session caregiver must correspond to the committing agent".into(),
        ));
    }

    if session.hours <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Session hours must be positive".into(),
        ));
    }
    if session.hours > 24.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Session cannot exceed 24 hours".into(),
        ));
    }
    if session.notes.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Session notes must be 4096 characters or fewer".into(),
        ));
    }
    if session.tasks_completed.len() > 50 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot list more than 50 tasks".into(),
        ));
    }
    for task in &session.tasks_completed {
        if task.len() > 256 {
            return Ok(ValidateCallbackResult::Invalid(
                "Each task must be 256 characters or fewer".into(),
            ));
        }
    }
    if session.ended_at <= session.started_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Session end must be after session start".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn create_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn update_action(author: AgentPubKey) -> Update {
        Update {
            author,
            timestamp: Timestamp::from_micros(1),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![9u8; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_plan(recipient: AgentPubKey, caregivers: Vec<AgentPubKey>) -> CarePlan {
        CarePlan {
            recipient,
            title: "Morning care".into(),
            description: "Assistance with morning routine".into(),
            care_type: CareType::Eldercare,
            schedule: "Mon/Wed/Fri mornings".into(),
            caregivers,
            status: PlanStatus::Draft,
            created_at: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
            hours_per_week: 6.0,
            special_instructions: String::new(),
        }
    }

    #[test]
    fn create_plan_valid_when_committer_is_recipient() {
        let plan = valid_plan(me(), vec![other_agent()]);
        let result = validate_create_plan(create_action(me()), plan).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_plan_valid_when_committer_is_caregiver() {
        let plan = valid_plan(other_agent(), vec![me()]);
        let result = validate_create_plan(create_action(me()), plan).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_plan_rejected_when_committer_is_neither() {
        let third_party = AgentPubKey::from_raw_36(vec![2u8; 36]);
        let plan = valid_plan(other_agent(), vec![third_party]);
        let result = validate_create_plan(create_action(me()), plan).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_session(caregiver: AgentPubKey) -> CareSession {
        CareSession {
            plan_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            caregiver,
            started_at: Timestamp::from_micros(0),
            ended_at: Timestamp::from_micros(1000),
            hours: 2.0,
            notes: String::new(),
            tasks_completed: vec![],
        }
    }

    #[test]
    fn create_session_valid_when_caregiver_matches_committer() {
        let s = valid_session(me());
        let result = validate_create_session(create_action(me()), s).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_session_forgery_rejected() {
        let s = valid_session(other_agent());
        let result = validate_create_session(create_action(me()), s).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_session_update() {
        let s = valid_session(me());
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::CareSession(s)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_plan_rejects_non_recipient_non_caregiver() {
        let plan = valid_plan(other_agent(), vec![other_agent()]);
        let third_party = AgentPubKey::from_raw_36(vec![2u8; 36]);
        let result = validate_update_plan(update_action(third_party), plan).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
