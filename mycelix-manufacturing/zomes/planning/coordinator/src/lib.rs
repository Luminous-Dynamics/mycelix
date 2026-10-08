// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Planning Coordinator Zome
//!
//! Material Requirements Planning (MRP) engine. Queries work orders, BOMs,
//! machine capacity, and optionally supplychain inventory via cross-cluster
//! `CallTargetCell::Local` calls within the manufacturing DNA, and
//! `CallTargetCell::OtherRole` for cross-cluster inventory lookups.

use hdk::prelude::*;
use planning_integrity::*;
use manufacturing_common::{
    select_capable_machine, CapabilityCandidate, CapabilityMismatch, CapabilityProfile,
    CapabilityQualification, CapabilityRequirement, MaterialShortage, MachineStatus, MrpResult,
    PlannedOrder,
};
use std::collections::HashMap;

/// Minimal projection of WorkOrderEntry for BOM linkage.
/// Only the fields we need, avoiding a dependency on workorders_integrity
/// (which would cause duplicate HDK symbol conflicts).
/// Uses `#[serde(default)]` on all fields so extra fields in WorkOrderEntry
/// are silently ignored during deserialization.
#[derive(Serialize, Deserialize, SerializedBytes, Debug)]
struct WorkOrderProjection {
    #[serde(default)]
    quantity: u64,
    #[serde(default)]
    bom_hash: Option<ActionHash>,
}

// ============================================================================
// Input / Output types
// ============================================================================

#[derive(Serialize, Deserialize, Debug)]
pub struct RunMrpInput {
    pub work_order_hashes: Vec<ActionHash>,
    pub horizon_days: Option<u32>,
    pub check_external_inventory: bool,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MrpOutput {
    pub mrp_run_hash: ActionHash,
    pub result: MrpResult,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct InventoryQueryInput {
    pub sku: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct InventoryLevel {
    pub sku: String,
    pub quantity: u64,
    pub source: String,
}

// ============================================================================
// Extern functions
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CapabilityPlanInput {
    pub requirement: CapabilityRequirement,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CapabilityPlanRejection {
    ContractNotFound,
    ContractMalformed,
    ContractLookupFailed,
    MachineNotFound,
    MachineLookupFailed,
    MachineStateAmbiguous,
    MachineInvalidRecord,
    MachineDeleted,
    MachineUnavailable,
    MultipleContractsForMachine,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CapabilityPlanDecision {
    pub machine_hash: Option<ActionHash>,
    pub capability_contract_hash: ActionHash,
    pub machine_status: Option<MachineStatus>,
    pub machine_state_head: Option<ActionHash>,
    pub eligible: bool,
    pub mismatch: Option<CapabilityMismatch>,
    pub rejection: Option<CapabilityPlanRejection>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CapabilityPlanSelection {
    pub selected_machine_hash: Option<ActionHash>,
    pub selected_capability_contract_hash: Option<ActionHash>,
    pub decisions: Vec<CapabilityPlanDecision>,
}

#[derive(Serialize, Deserialize, SerializedBytes, Debug, Clone)]
struct CapabilityContractProjection {
    machine_hash: ActionHash,
    process_family: String,
    material_classes: Vec<String>,
    envelope_x_mm: Option<u32>,
    envelope_y_mm: Option<u32>,
    envelope_z_mm: Option<u32>,
    tolerance_um: Option<u32>,
    supported_protocols: Vec<String>,
    qualification: CapabilityQualification,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
enum MachineStateResolutionProjection {
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

fn capability_profile(contract: &CapabilityContractProjection) -> CapabilityProfile {
    CapabilityProfile {
        process_family: contract.process_family.clone(),
        material_classes: contract.material_classes.clone(),
        envelope_x_mm: contract.envelope_x_mm,
        envelope_y_mm: contract.envelope_y_mm,
        envelope_z_mm: contract.envelope_z_mm,
        tolerance_um: contract.tolerance_um,
        supported_protocols: contract.supported_protocols.clone(),
        qualification: contract.qualification.clone(),
    }
}

/// Resolve a manufacturing capability requirement against live capability contracts
/// and the full machine update graph.
///
/// This is intentionally a resolver, not an execution authorization mechanism:
/// capability qualification is consumed exactly as recorded, machine availability
/// is evaluated separately, lookup failures remain distinct from missing records,
/// unvalidated or conflicting machine state fails closed, and ambiguous multiple
/// contracts for one machine fail closed.
#[hdk_extern]
pub fn select_live_capability(
    input: CapabilityPlanInput,
) -> ExternResult<CapabilityPlanSelection> {
    let response = call(
        CallTargetCell::Local,
        ZomeName::from("execution"),
        FunctionName::from("list_capability_contracts"),
        None,
        ExternIO::encode(())?,
    )?;

    let links = match response {
        ZomeCallResponse::Ok(data) => data.decode::<Vec<Link>>().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "failed to decode capability contract links: {e}"
            )))
        })?,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "failed to list capability contracts: {other:?}"
            )))
        }
    };

    let mut raw: Vec<(
        String,
        ActionHash,
        ActionHash,
        CapabilityProfile,
        MachineStatus,
        ActionHash,
    )> = Vec::new();
    let mut rejected: Vec<CapabilityPlanDecision> = Vec::new();

    for link in links {
        let Some(contract_hash) = link.target.clone().into_action_hash() else {
            continue;
        };

        let contract_response = call(
            CallTargetCell::Local,
            ZomeName::from("execution"),
            FunctionName::from("get_capability_contract"),
            None,
            ExternIO::encode(contract_hash.clone())?,
        )?;

        let contract_record = match contract_response {
            ZomeCallResponse::Ok(data) => match data.decode::<Option<Record>>().map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "failed to decode capability contract: {e}"
                )))
            })? {
                Some(record) => record,
                None => {
                    rejected.push(CapabilityPlanDecision {
                        machine_hash: None,
                        capability_contract_hash: contract_hash,
                        machine_status: None,
                        machine_state_head: None,
                        eligible: false,
                        mismatch: None,
                        rejection: Some(CapabilityPlanRejection::ContractNotFound),
                    });
                    continue;
                }
            },
            _ => {
                rejected.push(CapabilityPlanDecision {
                    machine_hash: None,
                    capability_contract_hash: contract_hash,
                    machine_status: None,
                    machine_state_head: None,
                    eligible: false,
                    mismatch: None,
                    rejection: Some(CapabilityPlanRejection::ContractLookupFailed),
                });
                continue;
            }
        };

        let contract: CapabilityContractProjection =
            match contract_record.entry().to_app_option().map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(e.to_string()))
            })? {
                Some(contract) => contract,
                None => {
                    rejected.push(CapabilityPlanDecision {
                        machine_hash: None,
                        capability_contract_hash: contract_hash,
                        machine_status: None,
                        machine_state_head: None,
                        eligible: false,
                        mismatch: None,
                        rejection: Some(CapabilityPlanRejection::ContractMalformed),
                    });
                    continue;
                }
            };

        let machine_hash = contract.machine_hash.clone();
        let machine_response = call(
            CallTargetCell::Local,
            ZomeName::from("machines"),
            FunctionName::from("get_current_machine_state"),
            None,
            ExternIO::encode(machine_hash.clone())?,
        )?;

        let resolution = match machine_response {
            ZomeCallResponse::Ok(data) => data
                .decode::<MachineStateResolutionProjection>()
                .map_err(|e| {
                    wasm_error!(WasmErrorInner::Guest(format!(
                        "failed to decode current machine state: {e}"
                    )))
                })?,
            _ => {
                rejected.push(CapabilityPlanDecision {
                    machine_hash: Some(machine_hash),
                    capability_contract_hash: contract_hash,
                    machine_status: None,
                    machine_state_head: None,
                    eligible: false,
                    mismatch: None,
                    rejection: Some(CapabilityPlanRejection::MachineLookupFailed),
                });
                continue;
            }
        };

        match resolution {
            MachineStateResolutionProjection::Resolved {
                status: MachineStatus::Available,
                head_action,
            } => {
                let profile = capability_profile(&contract);
                raw.push((
                    machine_hash.to_string(),
                    machine_hash.clone(),
                    contract_hash.clone(),
                    profile,
                    MachineStatus::Available,
                    head_action,
                ));
                continue;
            }
            MachineStateResolutionProjection::Resolved { status, head_action } => {
                rejected.push(CapabilityPlanDecision {
                    machine_hash: Some(machine_hash),
                    capability_contract_hash: contract_hash,
                    machine_status: Some(status),
                    machine_state_head: Some(head_action),
                    eligible: false,
                    mismatch: None,
                    rejection: Some(CapabilityPlanRejection::MachineUnavailable),
                });
                continue;
            }
            MachineStateResolutionProjection::NotFound => {
                rejected.push(CapabilityPlanDecision {
                    machine_hash: Some(machine_hash),
                    capability_contract_hash: contract_hash,
                    machine_status: None,
                    machine_state_head: None,
                    eligible: false,
                    mismatch: None,
                    rejection: Some(CapabilityPlanRejection::MachineNotFound),
                });
                continue;
            }
            MachineStateResolutionProjection::InvalidRecord => {
                rejected.push(CapabilityPlanDecision {
                    machine_hash: Some(machine_hash),
                    capability_contract_hash: contract_hash,
                    machine_status: None,
                    machine_state_head: None,
                    eligible: false,
                    mismatch: None,
                    rejection: Some(CapabilityPlanRejection::MachineInvalidRecord),
                });
                continue;
            }
            MachineStateResolutionProjection::Ambiguous { .. } => {
                rejected.push(CapabilityPlanDecision {
                    machine_hash: Some(machine_hash),
                    capability_contract_hash: contract_hash,
                    machine_status: None,
                    machine_state_head: None,
                    eligible: false,
                    mismatch: None,
                    rejection: Some(CapabilityPlanRejection::MachineStateAmbiguous),
                });
                continue;
            }
            MachineStateResolutionProjection::Deleted => {
                rejected.push(CapabilityPlanDecision {
                    machine_hash: Some(machine_hash),
                    capability_contract_hash: contract_hash,
                    machine_status: None,
                    machine_state_head: None,
                    eligible: false,
                    mismatch: None,
                    rejection: Some(CapabilityPlanRejection::MachineDeleted),
                });
                continue;
            }
        }


    }

    raw.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| a.2.to_string().cmp(&b.2.to_string()))
    });

    let mut machine_contract_counts = std::collections::HashMap::new();
    for (machine_id, _, _, _, _, _) in &raw {
        *machine_contract_counts.entry(machine_id.clone()).or_insert(0usize) += 1;
    }

    let mut candidates = Vec::new();
    let mut candidate_meta = Vec::new();

    for (machine_id, machine_hash, contract_hash, profile, status, head_action) in raw {
        if machine_contract_counts.get(&machine_id).copied().unwrap_or(0) != 1 {
            rejected.push(CapabilityPlanDecision {
                machine_hash: Some(machine_hash),
                capability_contract_hash: contract_hash,
                machine_status: Some(status),
                machine_state_head: Some(head_action),
                eligible: false,
                mismatch: None,
                rejection: Some(CapabilityPlanRejection::MultipleContractsForMachine),
            });
            continue;
        }

        candidates.push(CapabilityCandidate {
            machine_id: machine_id.clone(),
            profile,
        });
        candidate_meta.push((machine_id, machine_hash, contract_hash, status, head_action));
    }

    let selection = select_capable_machine(&input.requirement, candidates);

    let mut decisions = rejected;
    for decision in selection.decisions {
        if let Some((_, machine_hash, contract_hash, status, head_action)) = candidate_meta
            .iter()
            .find(|(id, _, _, _, _)| *id == decision.machine_id)
        {
            decisions.push(CapabilityPlanDecision {
                machine_hash: Some(machine_hash.clone()),
                capability_contract_hash: contract_hash.clone(),
                machine_status: Some(status.clone()),
                machine_state_head: Some(head_action.clone()),
                eligible: decision.eligible,
                mismatch: decision.mismatch,
                rejection: None,
            });
        }
    }

    decisions.sort_by(|a, b| {
        a.machine_hash
            .as_ref()
            .map(ToString::to_string)
            .cmp(&b.machine_hash.as_ref().map(ToString::to_string))
            .then_with(|| {
                a.capability_contract_hash
                    .to_string()
                    .cmp(&b.capability_contract_hash.to_string())
            })
    });

    let selected_machine_hash = selection.selected_machine_id.and_then(|id| {
        candidate_meta
            .iter()
            .find(|(machine_id, _, _, _, _)| *machine_id == id)
            .map(|(_, hash, _, _, _)| hash.clone())
    });
    let selected_capability_contract_hash = selected_machine_hash.as_ref().and_then(|hash| {
        candidate_meta
            .iter()
            .find(|(_, machine_hash, _, _, _)| machine_hash == hash)
            .map(|(_, _, contract_hash, _, _)| contract_hash.clone())
    });

    Ok(CapabilityPlanSelection {
        selected_machine_hash,
        selected_capability_contract_hash,
        decisions,
    })
}

/// Run an MRP planning cycle for the given work orders.
///
/// Steps:
/// 1. Fetch each work order via `CallTargetCell::Local` to the workorders zome
/// 2. For each work order, fetch the linked BOM via `CallTargetCell::Local` to bom zome
/// 3. Explode BOMs to get material requirements
/// 4. Optionally query supplychain inventory via `CallTargetCell::OtherRole`
/// 5. Compute shortages, planned orders, and machine schedule
/// 6. Persist the MRP run result
#[hdk_extern]
pub fn run_mrp(input: RunMrpInput) -> ExternResult<MrpOutput> {
    if input.work_order_hashes.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Must provide at least one work order hash".to_string()
        )));
    }

    let horizon = input.horizon_days.unwrap_or(30);
    if horizon == 0 || horizon > 365 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "horizon_days must be 1-365".to_string()
        )));
    }

    // --- Step 1: Fetch work orders via local zome call ---
    let mut material_needs: HashMap<String, u64> = HashMap::new();

    for wo_hash in &input.work_order_hashes {
        // Call workorders coordinator locally to get work order
        let response = call(
            CallTargetCell::Local,
            ZomeName::from("workorders"),
            FunctionName::from("get_work_order"),
            None,
            ExternIO::encode(wo_hash.clone())
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
        )?;
        let wo_record = match response {
            ZomeCallResponse::Ok(data) => {
                let record: Option<Record> = data.decode()
                    .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
                record.ok_or(wasm_error!(WasmErrorInner::Guest(format!(
                    "Work order not found: {:?}", wo_hash
                ))))?
            }
            other => return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to fetch work order: {:?}", other
            )))),
        };

        // --- Step 2-3: Fetch and explode BOM ---
        // Use a minimal projection struct to decode just the fields we need,
        // avoiding a dependency on workorders_integrity (HDK symbol conflicts).
        let wo_proj: Option<WorkOrderProjection> = wo_record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;

        if let Some(proj) = wo_proj {
            if let Some(bom_hash) = proj.bom_hash {
                let quantity = proj.quantity.max(1);

                // Call BOM coordinator to explode the BOM.
                let explode_input = ExternIO::encode(serde_json::json!({
                    "bom_hash": bom_hash,
                    "quantity": quantity,
                    "flatten": true
                }))
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;

                let bom_response = call(
                    CallTargetCell::Local,
                    ZomeName::from("bom"),
                    FunctionName::from("explode_bom"),
                    None,
                    explode_input,
                )?;

                if let ZomeCallResponse::Ok(data) = bom_response {
                    // Decode as a generic JSON value to extract explosion lines.
                    if let Ok(output) = data.decode::<serde_json::Value>() {
                        if let Some(lines) = output.get("lines").and_then(|l| l.as_array()) {
                            for line in lines {
                                if let (Some(part_id), Some(total_qty)) = (
                                    line.get("part_id").and_then(|p| p.as_str()),
                                    line.get("total_quantity").and_then(|q| q.as_u64()),
                                ) {
                                    *material_needs.entry(part_id.to_string()).or_insert(0) += total_qty;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    let material_needs: Vec<(String, u64)> = material_needs.into_iter().collect();

    // --- Step 4: Optionally query external inventory ---
    let mut external_inventory: Vec<InventoryLevel> = Vec::new();
    if input.check_external_inventory {
        // Query supplychain cluster via OtherRole bridge
        // This call resolves if the unified hApp has a supplychain role installed
        for (sku, _qty) in &material_needs {
            let query = InventoryQueryInput {
                sku: sku.clone(),
            };
            let payload = ExternIO::encode(query)
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
            match call(
                CallTargetCell::OtherRole("supplychain".into()),
                ZomeName::from("inventory"),
                FunctionName::from("get_stock_level_by_sku"),
                None,
                payload,
            ) {
                Ok(ZomeCallResponse::Ok(data)) => {
                    if let Ok(Some(level)) = data.decode::<Option<InventoryLevel>>() {
                        external_inventory.push(level);
                    }
                }
                _ => {
                    // Cross-cluster call failed -- supplychain may not be installed.
                    // Continue with local-only planning.
                }
            }
        }
    }

    // --- Step 5: Compute shortages and plan ---
    let mut shortages = Vec::new();
    let mut planned_orders = Vec::new();
    let now = sys_time()?;

    for (sku, qty_needed) in &material_needs {
        let available = external_inventory
            .iter()
            .find(|l| l.sku == *sku)
            .map(|l| l.quantity)
            .unwrap_or(0);

        if available < *qty_needed {
            let short = qty_needed - available;
            shortages.push(MaterialShortage {
                part_id: sku.clone(),
                quantity_needed: *qty_needed,
                quantity_available: available,
                short_quantity: short,
            });
            planned_orders.push(PlannedOrder {
                part_id: sku.clone(),
                quantity_needed: *qty_needed,
                quantity_available: available,
                quantity_to_order: short,
                due_date: now,
            });
        }
    }

    let feasible = shortages.is_empty();

    let result = MrpResult {
        planned_orders,
        scheduled_operations: Vec::new(), // TODO: machine scheduling
        capacity_warnings: Vec::new(),
        material_shortages: shortages,
        feasible,
    };

    // --- Step 6: Persist the MRP run ---
    let run_entry = MrpRunEntry {
        work_order_hashes: input.work_order_hashes.clone(),
        horizon_days: horizon,
        run_at: now,
        feasible,
    };
    let run_hash = create_entry(EntryTypes::MrpRun(run_entry))?;

    // Link from "all_mrp_runs" anchor
    let all_path = Path::from("all_mrp_runs").typed(LinkTypes::AllMrpRuns)?;
    all_path.ensure()?;
    create_link(
        all_path.path_entry_hash()?,
        run_hash.clone(),
        LinkTypes::AllMrpRuns,
        (),
    )?;

    // Link each work order to this MRP run
    for wo_hash in &input.work_order_hashes {
        create_link(
            wo_hash.clone(),
            run_hash.clone(),
            LinkTypes::WorkOrderToMrpRuns,
            (),
        )?;
    }

    // Persist shortage entries
    for shortage in &result.material_shortages {
        let entry = MaterialShortageEntry {
            mrp_run_hash: run_hash.clone(),
            part_id: shortage.part_id.clone(),
            quantity_needed: shortage.quantity_needed,
            quantity_available: shortage.quantity_available,
            short_quantity: shortage.short_quantity,
        };
        let sh = create_entry(EntryTypes::MaterialShortage(entry))?;
        create_link(
            run_hash.clone(),
            sh,
            LinkTypes::MrpRunToShortages,
            (),
        )?;
    }

    // Persist planned order entries
    for po in &result.planned_orders {
        let entry = PlannedOrderEntry {
            mrp_run_hash: run_hash.clone(),
            part_id: po.part_id.clone(),
            quantity_needed: po.quantity_needed,
            quantity_available: po.quantity_available,
            quantity_to_order: po.quantity_to_order,
            due_date: po.due_date,
        };
        let ph = create_entry(EntryTypes::PlannedOrder(entry))?;
        create_link(
            run_hash.clone(),
            ph,
            LinkTypes::MrpRunToPlannedOrders,
            (),
        )?;
    }

    Ok(MrpOutput {
        mrp_run_hash: run_hash,
        result,
    })
}

/// Get an MRP run by its action hash.
#[hdk_extern]
pub fn get_mrp_run(hash: ActionHash) -> ExternResult<Option<Record>> {
    get(hash, GetOptions::default())
}

/// List all MRP runs.
#[hdk_extern]
pub fn list_mrp_runs(_: ()) -> ExternResult<Vec<Link>> {
    let all_path = Path::from("all_mrp_runs").typed(LinkTypes::AllMrpRuns)?;
    get_links(
        GetLinksInputBuilder::try_new(all_path.path_entry_hash()?, LinkTypes::AllMrpRuns)?.build(),
    )
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_run_mrp_input_serde() {
        let input = RunMrpInput {
            work_order_hashes: vec![ActionHash::from_raw_36(vec![0u8; 36])],
            horizon_days: Some(30),
            check_external_inventory: true,
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: RunMrpInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.work_order_hashes.len(), 1);
        assert_eq!(back.horizon_days, Some(30));
        assert!(back.check_external_inventory);
    }

    #[test]
    fn test_inventory_level_serde() {
        let level = InventoryLevel {
            sku: "BOLT-M6".to_string(),
            quantity: 500,
            source: "supplychain".to_string(),
        };
        let json = serde_json::to_string(&level).unwrap();
        let back: InventoryLevel = serde_json::from_str(&json).unwrap();
        assert_eq!(back.sku, "BOLT-M6");
        assert_eq!(back.quantity, 500);
    }

    #[test]
    fn test_mrp_result_serde_roundtrip() {
        let result = MrpResult {
            planned_orders: vec![PlannedOrder {
                part_id: "P1".to_string(),
                quantity_needed: 100,
                quantity_available: 20,
                quantity_to_order: 80,
                due_date: Timestamp::from_micros(0),
            }],
            scheduled_operations: vec![],
            capacity_warnings: vec![],
            material_shortages: vec![MaterialShortage {
                part_id: "P1".to_string(),
                quantity_needed: 100,
                quantity_available: 20,
                short_quantity: 80,
            }],
            feasible: false,
        };
        let json = serde_json::to_string(&result).unwrap();
        let back: MrpResult = serde_json::from_str(&json).unwrap();
        assert!(!back.feasible);
        assert_eq!(back.material_shortages.len(), 1);
        assert_eq!(back.planned_orders[0].quantity_to_order, 80);
    }

    #[test]
    fn test_material_needs_accumulation() {
        // Simulate the HashMap merge logic used in run_mrp
        let mut material_needs: HashMap<String, u64> = HashMap::new();

        // First BOM explosion: BOLT-M6 x 10, NUT-M6 x 20
        *material_needs.entry("BOLT-M6".to_string()).or_insert(0) += 10;
        *material_needs.entry("NUT-M6".to_string()).or_insert(0) += 20;

        // Second BOM explosion: BOLT-M6 x 5, WASHER-M6 x 30
        *material_needs.entry("BOLT-M6".to_string()).or_insert(0) += 5;
        *material_needs.entry("WASHER-M6".to_string()).or_insert(0) += 30;

        assert_eq!(*material_needs.get("BOLT-M6").unwrap(), 15);
        assert_eq!(*material_needs.get("NUT-M6").unwrap(), 20);
        assert_eq!(*material_needs.get("WASHER-M6").unwrap(), 30);
        assert_eq!(material_needs.len(), 3);

        // Convert to Vec (as done in run_mrp)
        let needs_vec: Vec<(String, u64)> = material_needs.into_iter().collect();
        assert_eq!(needs_vec.len(), 3);
        let total: u64 = needs_vec.iter().map(|(_, q)| q).sum();
        assert_eq!(total, 65);
    }

    #[test]
    fn test_material_shortage_math() {
        let needed = 100u64;
        let available = 20u64;
        let short = needed.saturating_sub(available);
        assert_eq!(short, 80);
    }

    #[test]
    fn test_mrp_output_serde() {
        let output = MrpOutput {
            mrp_run_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            result: MrpResult {
                planned_orders: vec![],
                scheduled_operations: vec![],
                capacity_warnings: vec![],
                material_shortages: vec![],
                feasible: true,
            },
        };
        let json = serde_json::to_string(&output).unwrap();
        let back: MrpOutput = serde_json::from_str(&json).unwrap();
        assert!(back.result.feasible);
    }
}
