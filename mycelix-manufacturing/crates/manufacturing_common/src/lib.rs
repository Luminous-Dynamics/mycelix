// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Manufacturing Common Types
//!
//! Shared types and validation for the mycelix-manufacturing cluster.
//! All zomes in the cluster depend on this crate for canonical type definitions.

use hdi::prelude::*;
use serde::{Deserialize, Serialize};

// ============================================================================
// Work Orders
// ============================================================================

/// A manufacturing work order that tracks production of a specific product.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct WorkOrder {
    pub product_id: String,
    pub quantity: u64,
    pub due_date: Timestamp,
    pub status: WorkOrderStatus,
    pub priority: WorkOrderPriority,
    pub notes: Option<String>,
    pub bom_hash: Option<String>,
    pub routing_hash: Option<String>,
    pub created_at: Timestamp,
    pub updated_at: Timestamp,
}

/// Status lifecycle: Draft -> Released -> InProgress -> Completed | OnHold | Cancelled
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum WorkOrderStatus {
    Draft,
    Released,
    InProgress,
    Completed,
    OnHold,
    Cancelled,
    Closed,
}

impl WorkOrderStatus {
    /// Returns the set of valid next states from this status.
    pub fn valid_transitions(&self) -> Vec<WorkOrderStatus> {
        match self {
            Self::Draft => vec![Self::Released, Self::Cancelled],
            Self::Released => vec![Self::InProgress, Self::OnHold, Self::Cancelled],
            Self::InProgress => vec![Self::Completed, Self::OnHold, Self::Cancelled],
            Self::OnHold => vec![Self::Released, Self::InProgress, Self::Cancelled],
            Self::Completed => vec![Self::Closed],
            Self::Cancelled => vec![],
            Self::Closed => vec![],
        }
    }

    /// Check whether transitioning to `target` is valid from the current status.
    pub fn can_transition_to(&self, target: &WorkOrderStatus) -> bool {
        self.valid_transitions().contains(target)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum WorkOrderPriority {
    Low,
    Normal,
    High,
    Urgent,
}

// ============================================================================
// Bill of Materials (BOM)
// ============================================================================

/// A bill of materials for a product design.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BillOfMaterials {
    pub design_id: String,
    pub revision: String,
    pub items: Vec<BomItem>,
    pub created_at: Timestamp,
}

/// A single line item in a BOM.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BomItem {
    pub part_id: String,
    pub quantity_per: u64,
    pub unit: String,
    /// If set, this item is a sub-assembly referencing another BOM.
    pub sub_assembly_bom_hash: Option<String>,
    pub notes: Option<String>,
}

// ============================================================================
// Operations & Routing
// ============================================================================

/// A single manufacturing operation (e.g., "Mill profile", "Deburr").
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Operation {
    pub name: String,
    pub description: String,
    pub machine_type: String,
    pub setup_time_min: u32,
    pub cycle_time_min: u32,
    pub tooling: Option<String>,
}

/// An ordered sequence of operations that defines how a product is manufactured.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RoutingSequence {
    pub design_id: String,
    pub revision: String,
    pub steps: Vec<RoutingStep>,
    pub created_at: Timestamp,
}

/// A single step in a routing sequence.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RoutingStep {
    pub sequence: u32,
    pub operation: Operation,
}

// ============================================================================
// Machines
// ============================================================================

/// A registered manufacturing machine or workstation.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Machine {
    pub name: String,
    pub machine_type: MachineType,
    pub capabilities: Vec<String>,
    pub location: String,
    pub max_throughput_per_hour: u32,
    pub status: MachineStatus,
    pub current_work_order: Option<String>,
    pub registered_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MachineType {
    /// Fused Deposition Modeling (3D printing)
    FDM,
    /// Stereolithography (resin 3D printing)
    SLA,
    /// Selective Laser Sintering (powder 3D printing)
    SLS,
    /// 3-axis CNC milling
    CNC3Axis,
    /// 5-axis CNC milling
    CNC5Axis,
    /// Laser cutting / engraving
    LaserCutter,
    /// Manual or CNC lathe
    Lathe,
    /// Manual or automated assembly station
    Assembly,
    /// Custom / other machine type
    Custom(String),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MachineStatus {
    Available,
    Running,
    Maintenance,
    Offline,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CapabilityQualification {
    Declared,
    Observed,
    Verified,
    Qualified,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct CapabilityRequirement {
    pub process_family: String,
    pub material_class: String,
    pub envelope_x_mm: Option<u32>,
    pub envelope_y_mm: Option<u32>,
    pub envelope_z_mm: Option<u32>,
    /// Maximum permissible tolerance in micrometres. Smaller is stricter.
    pub tolerance_um: Option<u32>,
    pub required_protocols: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct CapabilityProfile {
    pub process_family: String,
    pub material_classes: Vec<String>,
    pub envelope_x_mm: Option<u32>,
    pub envelope_y_mm: Option<u32>,
    pub envelope_z_mm: Option<u32>,
    /// Smallest reliably qualified tolerance in micrometres.
    pub tolerance_um: Option<u32>,
    pub supported_protocols: Vec<String>,
    pub qualification: CapabilityQualification,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CapabilityMismatch {
    Unqualified,
    ProcessFamily,
    MaterialClass,
    EnvelopeX,
    EnvelopeY,
    EnvelopeZ,
    Tolerance,
    Protocol,
}

pub fn evaluate_capability(
    requirement: &CapabilityRequirement,
    profile: &CapabilityProfile,
) -> Result<(), CapabilityMismatch> {
    if !matches!(profile.qualification, CapabilityQualification::Qualified) {
        return Err(CapabilityMismatch::Unqualified);
    }

    if requirement.process_family != profile.process_family {
        return Err(CapabilityMismatch::ProcessFamily);
    }

    if !profile
        .material_classes
        .iter()
        .any(|material| material == &requirement.material_class)
    {
        return Err(CapabilityMismatch::MaterialClass);
    }

    if let Some(required) = requirement.envelope_x_mm {
        if profile.envelope_x_mm.is_none_or(|available| available < required) {
            return Err(CapabilityMismatch::EnvelopeX);
        }
    }

    if let Some(required) = requirement.envelope_y_mm {
        if profile.envelope_y_mm.is_none_or(|available| available < required) {
            return Err(CapabilityMismatch::EnvelopeY);
        }
    }

    if let Some(required) = requirement.envelope_z_mm {
        if profile.envelope_z_mm.is_none_or(|available| available < required) {
            return Err(CapabilityMismatch::EnvelopeZ);
        }
    }

    if let Some(required) = requirement.tolerance_um {
        if profile.tolerance_um.is_none_or(|available| available > required) {
            return Err(CapabilityMismatch::Tolerance);
        }
    }

    for protocol in &requirement.required_protocols {
        if !profile.supported_protocols.iter().any(|p| p == protocol) {
            return Err(CapabilityMismatch::Protocol);
        }
    }

    Ok(())
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct CapabilityCandidate {
    pub machine_id: String,
    pub profile: CapabilityProfile,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct CapabilityDecision {
    pub machine_id: String,
    pub eligible: bool,
    pub mismatch: Option<CapabilityMismatch>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct CapabilitySelection {
    pub selected_machine_id: Option<String>,
    pub decisions: Vec<CapabilityDecision>,
}

/// Deterministically assess candidates in stable machine-id order and select
/// the lexicographically first eligible machine. Network order and time do
/// not participate in the decision.
pub fn select_capable_machine(
    requirement: &CapabilityRequirement,
    mut candidates: Vec<CapabilityCandidate>,
) -> CapabilitySelection {
    candidates.sort_by(|a, b| a.machine_id.cmp(&b.machine_id));

    let mut selected_machine_id = None;
    let mut decisions = Vec::with_capacity(candidates.len());

    for candidate in candidates {
        match evaluate_capability(requirement, &candidate.profile) {
            Ok(()) => {
                let eligible = selected_machine_id.is_none();
                if eligible {
                    selected_machine_id = Some(candidate.machine_id.clone());
                }
                decisions.push(CapabilityDecision {
                    machine_id: candidate.machine_id,
                    eligible: true,
                    mismatch: None,
                });
            }
            Err(mismatch) => decisions.push(CapabilityDecision {
                machine_id: candidate.machine_id,
                eligible: false,
                mismatch: Some(mismatch),
            }),
        }
    }

    CapabilitySelection {
        selected_machine_id,
        decisions,
    }
}

impl MachineStatus {
    pub fn can_transition_to(&self, target: &MachineStatus) -> bool {
        match self {
            Self::Available => matches!(target, Self::Running | Self::Maintenance | Self::Offline),
            Self::Running => matches!(target, Self::Available | Self::Maintenance | Self::Offline),
            Self::Maintenance => matches!(target, Self::Available | Self::Offline),
            Self::Offline => matches!(target, Self::Available | Self::Maintenance),
        }
    }
}

// ============================================================================
// MRP (Material Requirements Planning)
// ============================================================================

/// Semantic state of MRP feasibility.
///
/// Material sufficiency is intentionally distinct from manufacturing schedule
/// feasibility. A plan cannot become fully feasible merely because no material
/// shortage was observed.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MrpFeasibility {
    MaterialInfeasible,
    SchedulingNotEvaluated,
    SchedulingInfeasible,
    Feasible,
}

impl Default for MrpFeasibility {
    fn default() -> Self {
        Self::SchedulingNotEvaluated
    }
}

/// Result of an MRP planning run.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MrpResult {
    pub planned_orders: Vec<PlannedOrder>,
    pub scheduled_operations: Vec<ScheduledOperation>,
    pub capacity_warnings: Vec<CapacityWarning>,
    pub material_shortages: Vec<MaterialShortage>,
    /// Full manufacturing feasibility, not merely material sufficiency.
    #[serde(default)]
    pub feasibility: MrpFeasibility,
}

impl MrpResult {
    pub fn full_feasible(&self) -> bool {
        matches!(self.feasibility, MrpFeasibility::Feasible)
    }
}

/// A planned procurement or production order generated by MRP.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PlannedOrder {
    pub part_id: String,
    pub quantity_needed: u64,
    pub quantity_available: u64,
    pub quantity_to_order: u64,
    pub due_date: Timestamp,
}

/// A scheduled operation assigned to a machine with time windows.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ScheduledOperation {
    pub work_order_id: String,
    pub operation_name: String,
    pub machine_id: String,
    pub start_time: Timestamp,
    pub end_time: Timestamp,
    pub setup_minutes: u32,
    pub run_minutes: u32,
}

/// A warning that machine capacity may be exceeded.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CapacityWarning {
    pub machine_id: String,
    pub machine_name: String,
    pub utilization_pct: f64,
    pub period: String,
    pub message: String,
}

/// A material shortage detected during MRP planning.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MaterialShortage {
    pub part_id: String,
    pub quantity_needed: u64,
    pub quantity_available: u64,
    pub short_quantity: u64,
}

// ============================================================================
// Validation helpers
// ============================================================================

/// Validate a work order has required fields and sane values.
pub fn validate_work_order(wo: &WorkOrder) -> Result<(), String> {
    if wo.product_id.is_empty() {
        return Err("product_id is required".to_string());
    }
    if wo.product_id.len() > 200 {
        return Err("product_id must be <= 200 characters".to_string());
    }
    if wo.quantity == 0 {
        return Err("quantity must be > 0".to_string());
    }
    if wo.quantity > 1_000_000 {
        return Err("quantity must be <= 1,000,000".to_string());
    }
    Ok(())
}

/// Validate a BOM has at least one item and valid fields.
pub fn validate_bom(bom: &BillOfMaterials) -> Result<(), String> {
    if bom.design_id.is_empty() {
        return Err("design_id is required".to_string());
    }
    if bom.revision.is_empty() {
        return Err("revision is required".to_string());
    }
    if bom.items.is_empty() {
        return Err("BOM must have at least one item".to_string());
    }
    for (i, item) in bom.items.iter().enumerate() {
        if item.part_id.is_empty() && item.sub_assembly_bom_hash.is_none() {
            return Err(format!("item {i}: part_id or sub_assembly_bom_hash required"));
        }
        if item.quantity_per == 0 {
            return Err(format!("item {i}: quantity_per must be > 0"));
        }
    }
    Ok(())
}

/// Validate a machine has required fields.
pub fn validate_machine(machine: &Machine) -> Result<(), String> {
    if machine.name.is_empty() {
        return Err("machine name is required".to_string());
    }
    if machine.name.len() > 200 {
        return Err("machine name must be <= 200 characters".to_string());
    }
    if machine.location.is_empty() {
        return Err("machine location is required".to_string());
    }
    if machine.max_throughput_per_hour == 0 {
        return Err("max_throughput_per_hour must be > 0".to_string());
    }
    Ok(())
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod capability_tests {
    use super::*;

    fn qualified_cnc() -> CapabilityProfile {
        CapabilityProfile {
            process_family: "milling".into(),
            material_classes: vec!["aluminum".into(), "plastic".into()],
            envelope_x_mm: Some(500),
            envelope_y_mm: Some(300),
            envelope_z_mm: Some(250),
            tolerance_um: Some(25),
            supported_protocols: vec!["opcua".into(), "mtconnect".into()],
            qualification: CapabilityQualification::Qualified,
        }
    }

    fn aluminum_requirement() -> CapabilityRequirement {
        CapabilityRequirement {
            process_family: "milling".into(),
            material_class: "aluminum".into(),
            envelope_x_mm: Some(400),
            envelope_y_mm: Some(200),
            envelope_z_mm: Some(100),
            tolerance_um: Some(50),
            required_protocols: vec!["opcua".into()],
        }
    }

    #[test]
    fn accepts_matching_qualified_capability() {
        assert!(evaluate_capability(&aluminum_requirement(), &qualified_cnc()).is_ok());
    }

    #[test]
    fn rejects_unqualified_profile_before_other_matches() {
        let mut profile = qualified_cnc();
        profile.qualification = CapabilityQualification::Verified;
        assert_eq!(
            evaluate_capability(&aluminum_requirement(), &profile),
            Err(CapabilityMismatch::Unqualified)
        );
    }

    #[test]
    fn rejects_process_family_mismatch() {
        let mut requirement = aluminum_requirement();
        requirement.process_family = "turning".into();
        assert_eq!(
            evaluate_capability(&requirement, &qualified_cnc()),
            Err(CapabilityMismatch::ProcessFamily)
        );
    }

    #[test]
    fn rejects_tight_tolerance_machine() {
        let mut profile = qualified_cnc();
        profile.tolerance_um = Some(75);
        let requirement = aluminum_requirement();
        assert_eq!(
            evaluate_capability(&requirement, &profile),
            Err(CapabilityMismatch::Tolerance)
        );
    }

    #[test]
    fn rejects_missing_protocol() {
        let mut requirement = aluminum_requirement();
        requirement.required_protocols = vec!["opcua".into(), "profinet".into()];
        assert_eq!(
            evaluate_capability(&requirement, &qualified_cnc()),
            Err(CapabilityMismatch::Protocol)
        );
    }

    #[test]
    fn selects_lexicographically_first_eligible_machine() {
        let selection = select_capable_machine(
            &aluminum_requirement(),
            vec![
                CapabilityCandidate {
                    machine_id: "CNC-02".into(),
                    profile: qualified_cnc(),
                },
                CapabilityCandidate {
                    machine_id: "CNC-01".into(),
                    profile: qualified_cnc(),
                },
            ],
        );

        assert_eq!(selection.selected_machine_id.as_deref(), Some("CNC-01"));
        assert_eq!(selection.decisions.len(), 2);
        assert!(selection.decisions.iter().all(|decision| decision.eligible));
    }

    #[test]
    fn selection_retains_rejection_reason() {
        let mut bad = qualified_cnc();
        bad.tolerance_um = Some(100);

        let selection = select_capable_machine(
            &aluminum_requirement(),
            vec![
                CapabilityCandidate {
                    machine_id: "CNC-BAD".into(),
                    profile: bad,
                },
                CapabilityCandidate {
                    machine_id: "CNC-GOOD".into(),
                    profile: qualified_cnc(),
                },
            ],
        );

        assert_eq!(selection.selected_machine_id.as_deref(), Some("CNC-GOOD"));
        assert_eq!(
            selection.decisions[0].mismatch,
            Some(CapabilityMismatch::Tolerance)
        );
        assert!(!selection.decisions[0].eligible);
        assert!(selection.decisions[1].eligible);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_timestamp() -> Timestamp {
        Timestamp::from_micros(1_700_000_000_000_000)
    }

    fn make_work_order(product_id: &str, qty: u64) -> WorkOrder {
        WorkOrder {
            product_id: product_id.to_string(),
            quantity: qty,
            due_date: test_timestamp(),
            status: WorkOrderStatus::Draft,
            priority: WorkOrderPriority::Normal,
            notes: None,
            bom_hash: None,
            routing_hash: None,
            created_at: test_timestamp(),
            updated_at: test_timestamp(),
        }
    }

    #[test]
    fn test_work_order_status_transitions() {
        let draft = WorkOrderStatus::Draft;
        assert!(draft.can_transition_to(&WorkOrderStatus::Released));
        assert!(draft.can_transition_to(&WorkOrderStatus::Cancelled));
        assert!(!draft.can_transition_to(&WorkOrderStatus::Completed));
        assert!(!draft.can_transition_to(&WorkOrderStatus::InProgress));
    }

    #[test]
    fn test_status_completed_only_to_closed() {
        let completed = WorkOrderStatus::Completed;
        assert!(completed.can_transition_to(&WorkOrderStatus::Closed));
        assert!(!completed.can_transition_to(&WorkOrderStatus::Draft));
        assert!(!completed.can_transition_to(&WorkOrderStatus::Released));
    }

    #[test]
    fn test_terminal_states_no_transitions() {
        assert!(WorkOrderStatus::Cancelled.valid_transitions().is_empty());
        assert!(WorkOrderStatus::Closed.valid_transitions().is_empty());
    }

    #[test]
    fn test_on_hold_transitions() {
        let hold = WorkOrderStatus::OnHold;
        assert!(hold.can_transition_to(&WorkOrderStatus::Released));
        assert!(hold.can_transition_to(&WorkOrderStatus::InProgress));
        assert!(hold.can_transition_to(&WorkOrderStatus::Cancelled));
        assert!(!hold.can_transition_to(&WorkOrderStatus::Completed));
    }

    #[test]
    fn test_validate_work_order_ok() {
        let wo = make_work_order("WIDGET-001", 100);
        assert!(validate_work_order(&wo).is_ok());
    }

    #[test]
    fn test_validate_work_order_empty_product() {
        let wo = make_work_order("", 100);
        assert_eq!(validate_work_order(&wo).unwrap_err(), "product_id is required");
    }

    #[test]
    fn test_validate_work_order_zero_qty() {
        let wo = make_work_order("W", 0);
        assert_eq!(validate_work_order(&wo).unwrap_err(), "quantity must be > 0");
    }

    #[test]
    fn test_validate_bom_ok() {
        let bom = BillOfMaterials {
            design_id: "D-001".to_string(),
            revision: "A".to_string(),
            items: vec![BomItem {
                part_id: "BOLT-M6".to_string(),
                quantity_per: 4,
                unit: "each".to_string(),
                sub_assembly_bom_hash: None,
                notes: None,
            }],
            created_at: test_timestamp(),
        };
        assert!(validate_bom(&bom).is_ok());
    }

    #[test]
    fn test_validate_bom_empty_items() {
        let bom = BillOfMaterials {
            design_id: "D-001".to_string(),
            revision: "A".to_string(),
            items: vec![],
            created_at: test_timestamp(),
        };
        assert!(validate_bom(&bom).unwrap_err().contains("at least one item"));
    }

    #[test]
    fn test_validate_machine_ok() {
        let m = Machine {
            name: "CNC Mill #1".to_string(),
            machine_type: MachineType::CNC3Axis,
            capabilities: vec!["milling".to_string()],
            location: "Bay 1".to_string(),
            max_throughput_per_hour: 10,
            status: MachineStatus::Available,
            current_work_order: None,
            registered_at: test_timestamp(),
        };
        assert!(validate_machine(&m).is_ok());
    }

    #[test]
    fn test_validate_machine_empty_name() {
        let m = Machine {
            name: String::new(),
            machine_type: MachineType::Lathe,
            capabilities: vec![],
            location: "Bay 1".to_string(),
            max_throughput_per_hour: 5,
            status: MachineStatus::Available,
            current_work_order: None,
            registered_at: test_timestamp(),
        };
        assert!(validate_machine(&m).unwrap_err().contains("name is required"));
    }

    #[test]
    fn test_machine_status_transitions() {
        let avail = MachineStatus::Available;
        assert!(avail.can_transition_to(&MachineStatus::Running));
        assert!(avail.can_transition_to(&MachineStatus::Maintenance));
        assert!(!avail.can_transition_to(&MachineStatus::Available));
    }

    #[test]
    fn test_serde_roundtrip_work_order() {
        let wo = make_work_order("TEST-001", 50);
        let json = serde_json::to_string(&wo).unwrap();
        let back: WorkOrder = serde_json::from_str(&json).unwrap();
        assert_eq!(back.product_id, "TEST-001");
        assert_eq!(back.quantity, 50);
    }

    #[test]
    fn test_serde_roundtrip_machine_type() {
        let custom = MachineType::Custom("Waterjet".to_string());
        let json = serde_json::to_string(&custom).unwrap();
        let back: MachineType = serde_json::from_str(&json).unwrap();
        assert_eq!(back, MachineType::Custom("Waterjet".to_string()));
    }

    #[test]
    fn test_mrp_feasibility_default_is_not_full_feasible() {
        let status = MrpFeasibility::default();
        assert_eq!(status, MrpFeasibility::SchedulingNotEvaluated);
        assert!(!matches!(status, MrpFeasibility::Feasible));
    }

    #[test]
    fn test_mrp_feasibility_serde_roundtrip() {
        for status in [
            MrpFeasibility::MaterialInfeasible,
            MrpFeasibility::SchedulingNotEvaluated,
            MrpFeasibility::SchedulingInfeasible,
            MrpFeasibility::Feasible,
        ] {
            let json = serde_json::to_string(&status).unwrap();
            let back: MrpFeasibility = serde_json::from_str(&json).unwrap();
            assert_eq!(back, status);
        }
    }

    #[test]
    fn test_mrp_result_construction() {
        let result = MrpResult {
            planned_orders: vec![],
            scheduled_operations: vec![],
            capacity_warnings: vec![],
            material_shortages: vec![MaterialShortage {
                part_id: "BOLT-M6".to_string(),
                quantity_needed: 100,
                quantity_available: 20,
                short_quantity: 80,
            }],
            feasibility: MrpFeasibility::MaterialInfeasible,
        };
        assert!(!result.full_feasible());
        assert_eq!(result.material_shortages[0].short_quantity, 80);
    }

    #[test]
    fn test_mrp_result_constructs_material_infeasible() {
        let result = MrpResult {
            planned_orders: vec![],
            scheduled_operations: vec![],
            capacity_warnings: vec![],
            material_shortages: vec![],
            feasibility: MrpFeasibility::SchedulingNotEvaluated,
            feasible: false,
        };
        assert_eq!(result.feasibility, MrpFeasibility::SchedulingNotEvaluated);
        assert!(!result.feasible);
    }


}
