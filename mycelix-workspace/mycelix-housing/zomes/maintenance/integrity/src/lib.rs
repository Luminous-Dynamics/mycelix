// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Maintenance Integrity Zome
//! Entry types and validation for maintenance requests, work orders, and inspections.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Category of maintenance issue
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MaintenanceCategory {
    Plumbing,
    Electrical,
    HVAC,
    Structural,
    Appliance,
    Exterior,
    CommonArea,
    Safety,
    Pest,
    Other(String),
}

/// Priority of a maintenance request
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MaintenancePriority {
    Emergency,
    Urgent,
    Normal,
    Low,
    Scheduled,
}

/// Status of a maintenance request
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MaintenanceStatus {
    Reported,
    Acknowledged,
    Scheduled,
    InProgress,
    Completed,
    Deferred,
}

/// A maintenance request
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MaintenanceRequest {
    pub unit_hash: Option<ActionHash>,
    pub building_hash: ActionHash,
    pub reported_by: AgentPubKey,
    pub title: String,
    pub description: String,
    pub category: MaintenanceCategory,
    pub priority: MaintenancePriority,
    pub status: MaintenanceStatus,
    pub reported_at: Timestamp,
}

/// A work order for maintenance
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WorkOrder {
    pub request_hash: ActionHash,
    pub assigned_to: String,
    pub description: String,
    pub estimated_cost_cents: Option<u64>,
    pub actual_cost_cents: Option<u64>,
    pub scheduled_date: Option<Timestamp>,
    pub completed_date: Option<Timestamp>,
    pub notes: String,
}

/// Type of building inspection
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum InspectionType {
    Annual,
    Safety,
    Code,
    PreMove,
    PostMove,
}

/// A building inspection record
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Inspection {
    pub building_hash: ActionHash,
    pub inspector: AgentPubKey,
    pub inspection_type: InspectionType,
    pub date: Timestamp,
    pub findings: Vec<String>,
    pub passed: bool,
    pub next_due: Option<Timestamp>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    MaintenanceRequest(MaintenanceRequest),
    WorkOrder(WorkOrder),
    Inspection(Inspection),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Open requests anchor
    OpenRequests,
    /// Building to maintenance requests
    BuildingToRequest,
    /// Request to work orders
    RequestToWorkOrder,
    /// Building to inspections
    BuildingToInspection,
    /// Reporter to their requests
    ReporterToRequest,
    /// All completed requests
    CompletedRequests,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::MaintenanceRequest(req) => validate_create_request(action, req),
                EntryTypes::WorkOrder(order) => validate_create_work_order(action, order),
                EntryTypes::Inspection(inspection) => {
                    validate_create_inspection(action, inspection)
                }
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
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            LinkTypes::OpenRequests => Ok(ValidateCallbackResult::Valid),
            LinkTypes::BuildingToRequest => Ok(ValidateCallbackResult::Valid),
            LinkTypes::RequestToWorkOrder => Ok(ValidateCallbackResult::Valid),
            LinkTypes::BuildingToInspection => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ReporterToRequest => Ok(ValidateCallbackResult::Valid),
            LinkTypes::CompletedRequests => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap):
        // complete_work_order deletes the OpenRequests link when a
        // request is completed, and completion is typically performed by
        // whoever finished the work, not necessarily the original
        // reporter (and thus not the link's original creator).
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 24th confirmed instance of this exact
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
        EntryTypes::MaintenanceRequest(req) => validate_update_request(action, req),
        EntryTypes::WorkOrder(order) => validate_update_work_order(action, order),
        EntryTypes::Inspection(inspection) => validate_update_inspection(action, inspection),
    }
}

/// No author requirement: acknowledge_request/create_work_order/
/// complete_work_order all change only `status`, with zero
/// caller-identity check in the coordinator (no assignee/steward role
/// concept exists here to bind against) -- case (c). Content restricted
/// to status only -- this closes the wide-open bug that previously let
/// reported_by/title/description/category/priority change
/// unconditionally on update too.
fn validate_update_request(
    action: Update,
    req: MaintenanceRequest,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: MaintenanceRequest = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original request not found".into()
        )))?;

    if req.unit_hash != original.unit_hash
        || req.building_hash != original.building_hash
        || req.reported_by != original.reported_by
        || req.title != original.title
        || req.description != original.description
        || req.category != original.category
        || req.priority != original.priority
        || req.reported_at != original.reported_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status can change on a maintenance request update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_request(
    action: Create,
    req: MaintenanceRequest,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's submit_request previously took
    // the FULL struct straight from caller input with ZERO derivation
    // from agent_info() -- any agent could forge a victim agent as
    // reporter. Found + fixed 2026-07-09 during the P0 author-binding
    // pass (coordinator-side fix applied alongside this).
    if req.reported_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Request reported_by must correspond to the committing agent".into(),
        ));
    }

    if req.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Request title cannot be empty".into(),
        ));
    }
    if req.title.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Request title must be at most 256 characters".into(),
        ));
    }
    if req.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Request description cannot be empty".into(),
        ));
    }
    if req.status != MaintenanceStatus::Reported {
        return Ok(ValidateCallbackResult::Invalid(
            "New requests must have Reported status".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_work_order(
    _action: Create,
    order: WorkOrder,
) -> ExternResult<ValidateCallbackResult> {
    if order.assigned_to.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Work order must be assigned to someone".into(),
        ));
    }
    if order.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Work order description cannot be empty".into(),
        ));
    }
    if order.completed_date.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New work orders cannot have a completed date".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// No author requirement: complete_work_order has zero caller-identity
/// check in the coordinator -- case (c). Content is restricted to
/// completed_date/actual_cost_cents/notes (the exact fields
/// complete_work_order changes) -- this closes the wide-open bug that
/// previously let assigned_to/request_hash/description/
/// estimated_cost_cents/scheduled_date change unconditionally on update
/// too.
fn validate_update_work_order(
    action: Update,
    order: WorkOrder,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: WorkOrder = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original work order not found".into()
        )))?;

    if order.request_hash != original.request_hash
        || order.assigned_to != original.assigned_to
        || order.description != original.description
        || order.estimated_cost_cents != original.estimated_cost_cents
        || order.scheduled_date != original.scheduled_date
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only completed_date/actual_cost_cents/notes can change on a work order update".into(),
        ));
    }

    if order.assigned_to.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Work order must be assigned to someone".into(),
        ));
    }
    if let (Some(actual), Some(estimated)) = (order.actual_cost_cents, order.estimated_cost_cents) {
        // Allow up to 200% of estimate without special approval
        if actual > estimated * 2 {
            return Ok(ValidateCallbackResult::Invalid(
                "Actual cost exceeds 200% of estimate; requires special approval".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_inspection(
    action: Create,
    inspection: Inspection,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's schedule_inspection previously
    // took the FULL struct straight from caller input with ZERO
    // derivation from agent_info() -- any agent could forge a victim
    // agent as inspector. Found + fixed 2026-07-09 during the P0
    // author-binding pass (coordinator-side fix applied alongside this).
    if inspection.inspector != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Inspection inspector must correspond to the committing agent".into(),
        ));
    }

    if inspection.findings.len() > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Maximum 100 findings per inspection".into(),
        ));
    }
    for finding in &inspection.findings {
        if finding.is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Inspection findings cannot be empty strings".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

/// No author requirement: record_inspection has zero caller-identity
/// check either -- case (c). Content restricted to
/// findings/passed/next_due.
fn validate_update_inspection(
    action: Update,
    inspection: Inspection,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Inspection = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original inspection not found".into()
        )))?;

    if inspection.building_hash != original.building_hash
        || inspection.inspector != original.inspector
        || inspection.inspection_type != original.inspection_type
        || inspection.date != original.date
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only findings/passed/next_due can change on an inspection update".into(),
        ));
    }

    if inspection.findings.len() > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Maximum 100 findings per inspection".into(),
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

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_request(reported_by: AgentPubKey) -> MaintenanceRequest {
        MaintenanceRequest {
            unit_hash: None,
            building_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            reported_by,
            title: "Leaky faucet".into(),
            description: "Kitchen faucet drips constantly".into(),
            category: MaintenanceCategory::Plumbing,
            priority: MaintenancePriority::Normal,
            status: MaintenanceStatus::Reported,
            reported_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_request_valid_when_reporter_matches_committer() {
        let r = valid_request(me());
        let result = validate_create_request(create_action(me()), r).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_request_forgery_rejected() {
        let r = valid_request(me());
        let result = validate_create_request(create_action(other_agent()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_inspection(inspector: AgentPubKey) -> Inspection {
        Inspection {
            building_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            inspector,
            inspection_type: InspectionType::Annual,
            date: Timestamp::from_micros(0),
            findings: vec![],
            passed: true,
            next_due: None,
        }
    }

    #[test]
    fn create_inspection_valid_when_inspector_matches_committer() {
        let i = valid_inspection(me());
        let result = validate_create_inspection(create_action(me()), i).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_inspection_forgery_rejected() {
        let i = valid_inspection(me());
        let result = validate_create_inspection(create_action(other_agent()), i).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
