// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Work Orders Integrity Zome
//!
//! Entry types and validation for manufacturing work orders.

use hdi::prelude::*;
use manufacturing_common::{WorkOrderPriority, WorkOrderStatus};

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WorkOrderEntry {
    pub product_id: String,
    pub quantity: u64,
    pub due_date: Timestamp,
    pub status: WorkOrderStatus,
    pub priority: WorkOrderPriority,
    pub notes: Option<String>,
    pub bom_hash: Option<ActionHash>,
    pub routing_hash: Option<ActionHash>,
    pub created_at: Timestamp,
    pub updated_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WorkOrderStatusUpdate {
    pub work_order_hash: ActionHash,
    pub previous_status: WorkOrderStatus,
    pub new_status: WorkOrderStatus,
    pub updated_at: Timestamp,
    pub reason: Option<String>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    WorkOrder(WorkOrderEntry),
    StatusUpdate(WorkOrderStatusUpdate),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllWorkOrders,
    ProductToWorkOrders,
    WorkOrderToStatusUpdates,
    StatusToWorkOrders,
}

/// **P0 author-binding pass, 2026-07-09**: `WorkOrder`/`StatusUpdate` have
/// no identity field at all (case a -- this is a shared shop-floor
/// system, not per-agent-owned data), so no author-binding is possible.
/// What IS fixed: `OpEntry::UpdateEntry` and `FlatOp::RegisterUpdate` were
/// both routed through the wide-open catch-all `_ => Valid` -- meaning
/// `update_work_order_status`'s `update_entry` call on WorkOrder was
/// accepted with ZERO validation, not even the create-path's own
/// non-empty/positive-quantity checks, and a modified coordinator could
/// silently rewrite ANY field (product_id, quantity, due_date, bom_hash,
/// routing_hash), not just the intended status/updated_at. Fixed via
/// must_get content-restriction plus re-running the same
/// `can_transition_to` state-machine check the create-path already
/// enforces for StatusUpdate.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, .. }) => {
            validate_create_entry(app_entry)
        }
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry,
            original_action_hash,
            ..
        }) => validate_update_entry(original_action_hash, app_entry),
        FlatOp::RegisterUpdate(OpUpdate::Entry {
            app_entry, action, ..
        }) => validate_update_entry(action.original_action_address, app_entry),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_entry(entry: EntryTypes) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::WorkOrder(wo) => {
            if wo.product_id.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "product_id is required".into(),
                ));
            }
            if wo.quantity == 0 {
                return Ok(ValidateCallbackResult::Invalid(
                    "quantity must be > 0".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::StatusUpdate(su) => {
            if !su.previous_status.can_transition_to(&su.new_status) {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Invalid transition: {:?} -> {:?}",
                    su.previous_status, su.new_status
                )));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

fn validate_update_entry(
    original_action_hash: ActionHash,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::WorkOrder(wo) => {
            let original_record = must_get_valid_record(original_action_hash)?;
            let original: WorkOrderEntry = original_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Original work order not found".into()
                )))?;

            if wo.product_id != original.product_id
                || wo.quantity != original.quantity
                || wo.due_date != original.due_date
                || wo.bom_hash != original.bom_hash
                || wo.routing_hash != original.routing_hash
                || wo.created_at != original.created_at
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only status/priority/notes/updated_at can change on a work order update"
                        .into(),
                ));
            }

            if !original.status.can_transition_to(&wo.status) {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Invalid transition: {:?} -> {:?}",
                    original.status, wo.status
                )));
            }

            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::StatusUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Status update records are immutable".into(),
        )),
    }
}

#[cfg(test)]
mod content_restriction_tests {
    use super::*;

    fn valid_work_order() -> WorkOrderEntry {
        WorkOrderEntry {
            product_id: "prod-1".into(),
            quantity: 10,
            due_date: Timestamp::from_micros(0),
            status: WorkOrderStatus::Draft,
            priority: WorkOrderPriority::Normal,
            notes: None,
            bom_hash: None,
            routing_hash: None,
            created_at: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_work_order_requires_product_id() {
        let mut wo = valid_work_order();
        wo.product_id = "".into();
        let result = validate_create_entry(EntryTypes::WorkOrder(wo)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_work_order_requires_positive_quantity() {
        let mut wo = valid_work_order();
        wo.quantity = 0;
        let result = validate_create_entry(EntryTypes::WorkOrder(wo)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_work_order_valid() {
        let result = validate_create_entry(EntryTypes::WorkOrder(valid_work_order())).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn status_update_rejects_invalid_transition() {
        let su = WorkOrderStatusUpdate {
            work_order_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            previous_status: WorkOrderStatus::Draft,
            new_status: WorkOrderStatus::Completed,
            updated_at: Timestamp::from_micros(0),
            reason: None,
        };
        let result = validate_create_entry(EntryTypes::StatusUpdate(su)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // validate_update_entry calls must_get_valid_record, which requires a
    // live HDI host and can't run in a plain unit test -- matching the
    // established pattern from every other zome's update validator this
    // pass.
}
