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
// Versioned external manufacturing-contract references
// ============================================================================

/// Exact public source location for the Symthaea manufacturing contract API.
/// This is pinned to the source commit inspected by MFG-PROC-001A; it is not
/// proof that the source object has been fetched, authenticated, or qualified.
pub const SYMTHAEA_MANUFACTURING_CONTRACT_REPOSITORY: &str = "Luminous-Dynamics/symthaea";
pub const SYMTHAEA_MANUFACTURING_CONTRACT_COMMIT_V1: &str =
    "0b3feb49a3c2e9c1c1748ca2a0c0ff40ed320ecd";
pub const SYMTHAEA_PROCESS_CRATE: &str = "symthaea-manufacturing-process";
pub const SYMTHAEA_CONTRACTS_CRATE: &str = "symthaea-manufacturing-contracts";

/// Immutable coordinate of an object in the canonical Symthaea contract API.
/// Object IDs use the BLAKE3 identifiers defined by that API. The source
/// commit pins the API implementation; it does not authenticate the object.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct SymthaeaManufacturingObjectRefV1 {
    pub owner_repository: String,
    pub owner_commit: String,
    pub contract_crate: String,
    pub contract_type: String,
    pub object_id_blake3: String,
    pub process_id_blake3: String,
    pub semantic_version: String,
}

impl SymthaeaManufacturingObjectRefV1 {
    pub fn validate(&self, expected_crate: &str, expected_type: &str) -> Result<(), String> {
        if self.owner_repository != SYMTHAEA_MANUFACTURING_CONTRACT_REPOSITORY {
            return Err("manufacturing reference has an unexpected owner repository".into());
        }
        if self.owner_commit != SYMTHAEA_MANUFACTURING_CONTRACT_COMMIT_V1
            || !is_lower_hex_digest(&self.owner_commit, 40)
        {
            return Err(
                "manufacturing reference is not pinned to the supported source commit".into(),
            );
        }
        if self.contract_crate != expected_crate || self.contract_type != expected_type {
            return Err("manufacturing reference has the wrong canonical contract type".into());
        }
        if !is_lower_hex_digest(&self.object_id_blake3, 64)
            || !is_lower_hex_digest(&self.process_id_blake3, 64)
        {
            return Err("manufacturing reference IDs must be lowercase BLAKE3 digests".into());
        }
        if self.semantic_version.is_empty()
            || self.semantic_version.len() > 64
            || self.semantic_version.trim() != self.semantic_version
            || self.semantic_version.chars().any(char::is_control)
        {
            return Err("manufacturing reference semantic_version is invalid".into());
        }
        Ok(())
    }
}

fn is_lower_hex_digest(value: &str, expected_len: usize) -> bool {
    value.len() == expected_len
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(transparent)]
pub struct ProcessDefinitionRefV1(pub SymthaeaManufacturingObjectRefV1);

impl ProcessDefinitionRefV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.0
            .validate(SYMTHAEA_PROCESS_CRATE, "ProcessDefinitionV1")?;
        if self.0.object_id_blake3 != self.0.process_id_blake3 {
            return Err("process definition object ID must equal its process ID".into());
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(transparent)]
pub struct ProcessCapabilityRequirementRefV1(pub SymthaeaManufacturingObjectRefV1);

impl ProcessCapabilityRequirementRefV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.0
            .validate(SYMTHAEA_CONTRACTS_CRATE, "ProcessCapabilityRequirementV1")
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(transparent)]
pub struct ProcessCapabilityProfileRefV1(pub SymthaeaManufacturingObjectRefV1);

impl ProcessCapabilityProfileRefV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.0
            .validate(SYMTHAEA_CONTRACTS_CRATE, "ProcessCapabilityProfileV1")
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(transparent)]
pub struct ProcessRecipeRefV1(pub SymthaeaManufacturingObjectRefV1);

impl ProcessRecipeRefV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.0.validate(SYMTHAEA_CONTRACTS_CRATE, "ProcessRecipeV1")
    }
}

/// Epistemic state of a reference into an external process contract.
/// v0.1 can classify only legacy, syntactically present, and malformed refs;
/// Unavailable/Verified require a future exact owner-resolution adapter.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcessReferenceResolutionV1 {
    LegacyUnbound,
    TypedReferencePresent,
    TypedReferenceMalformed,
    OwnerObjectUnavailable,
    OwnerObjectVerified,
}

/// A typed routing binding to canonical process semantics. The legacy
/// machine_type string remains descriptive compatibility metadata only.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProcessBindingV1 {
    pub process_definition: ProcessDefinitionRefV1,
    pub capability_requirement: ProcessCapabilityRequirementRefV1,
    pub recipe: Option<ProcessRecipeRefV1>,
}

impl ProcessBindingV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.process_definition.validate()?;
        self.capability_requirement.validate()?;
        if self.process_definition.0.process_id_blake3
            != self.capability_requirement.0.process_id_blake3
        {
            return Err("process definition and capability requirement IDs do not match".into());
        }
        if self.process_definition.0.owner_commit != self.capability_requirement.0.owner_commit {
            return Err("process binding mixes source contract commits".into());
        }
        if let Some(recipe) = &self.recipe {
            recipe.validate()?;
            if recipe.0.process_id_blake3 != self.process_definition.0.process_id_blake3
                || recipe.0.owner_commit != self.process_definition.0.owner_commit
            {
                return Err("recipe does not match the bound process definition/commit".into());
            }
        }
        Ok(())
    }

    /// Classify only local structure. Presence is not proof that the external
    /// object exists or has been authenticated.
    pub fn resolution_state(&self) -> ProcessReferenceResolutionV1 {
        if self.validate().is_ok() {
            ProcessReferenceResolutionV1::TypedReferencePresent
        } else {
            ProcessReferenceResolutionV1::TypedReferenceMalformed
        }
    }
}

pub fn process_binding_resolution(
    binding: Option<&ProcessBindingV1>,
) -> ProcessReferenceResolutionV1 {
    match binding {
        None => ProcessReferenceResolutionV1::LegacyUnbound,
        Some(binding) => binding.resolution_state(),
    }
}

/// Typed process-capability reference on a machine. It is still only an
/// external claim/reference; it does not mean the profile has been verified
/// or that the machine is qualified for production.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineProcessCapabilityRefV1 {
    pub capability_profile: ProcessCapabilityProfileRefV1,
}

impl MachineProcessCapabilityRefV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.capability_profile.validate()
    }
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

/// Result of an MRP planning run.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MrpResult {
    pub planned_orders: Vec<PlannedOrder>,
    pub scheduled_operations: Vec<ScheduledOperation>,
    pub capacity_warnings: Vec<CapacityWarning>,
    pub material_shortages: Vec<MaterialShortage>,
    pub feasible: bool,
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
            return Err(format!(
                "item {i}: part_id or sub_assembly_bom_hash required"
            ));
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
        assert_eq!(
            validate_work_order(&wo).unwrap_err(),
            "product_id is required"
        );
    }

    #[test]
    fn test_validate_work_order_zero_qty() {
        let wo = make_work_order("W", 0);
        assert_eq!(
            validate_work_order(&wo).unwrap_err(),
            "quantity must be > 0"
        );
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
        assert!(
            validate_bom(&bom)
                .unwrap_err()
                .contains("at least one item")
        );
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
        assert!(
            validate_machine(&m)
                .unwrap_err()
                .contains("name is required")
        );
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

    fn manufacturing_ref(
        kind: &str,
        crate_name: &str,
        object_id: char,
    ) -> SymthaeaManufacturingObjectRefV1 {
        let digest = |ch: char, count: usize| std::iter::repeat_n(ch, count).collect::<String>();
        SymthaeaManufacturingObjectRefV1 {
            owner_repository: SYMTHAEA_MANUFACTURING_CONTRACT_REPOSITORY.into(),
            owner_commit: SYMTHAEA_MANUFACTURING_CONTRACT_COMMIT_V1.into(),
            contract_crate: crate_name.into(),
            contract_type: kind.into(),
            object_id_blake3: digest(object_id, 64),
            process_id_blake3: digest('a', 64),
            semantic_version: "1.0.0".into(),
        }
    }

    #[test]
    fn reference_resolution_never_upgrades_syntax_to_owner_verification() {
        assert_eq!(
            process_binding_resolution(None),
            ProcessReferenceResolutionV1::LegacyUnbound
        );
        let process = ProcessDefinitionRefV1(manufacturing_ref(
            "ProcessDefinitionV1",
            SYMTHAEA_PROCESS_CRATE,
            'a',
        ));
        let binding = ProcessBindingV1 {
            process_definition: process,
            capability_requirement: ProcessCapabilityRequirementRefV1(manufacturing_ref(
                "ProcessCapabilityRequirementV1",
                SYMTHAEA_CONTRACTS_CRATE,
                'b',
            )),
            recipe: None,
        };
        assert_eq!(
            process_binding_resolution(Some(&binding)),
            ProcessReferenceResolutionV1::TypedReferencePresent
        );
        assert_ne!(
            process_binding_resolution(Some(&binding)),
            ProcessReferenceResolutionV1::OwnerObjectVerified
        );
    }

    #[test]
    fn typed_process_definition_ref_requires_exact_pinned_owner_and_type() {
        let mut reference = manufacturing_ref("ProcessDefinitionV1", SYMTHAEA_PROCESS_CRATE, 'a');
        let typed = ProcessDefinitionRefV1(reference.clone());
        assert!(typed.validate().is_ok());

        reference.owner_commit = "f".repeat(40);
        assert!(ProcessDefinitionRefV1(reference).validate().is_err());
    }

    #[test]
    fn typed_process_binding_rejects_mismatched_process_identity() {
        let process = ProcessDefinitionRefV1(manufacturing_ref(
            "ProcessDefinitionV1",
            SYMTHAEA_PROCESS_CRATE,
            'a',
        ));
        let mut requirement_ref = manufacturing_ref(
            "ProcessCapabilityRequirementV1",
            SYMTHAEA_CONTRACTS_CRATE,
            'b',
        );
        requirement_ref.process_id_blake3 = "c".repeat(64);
        let binding = ProcessBindingV1 {
            process_definition: process,
            capability_requirement: ProcessCapabilityRequirementRefV1(requirement_ref),
            recipe: None,
        };
        assert!(binding.validate().is_err());
    }

    #[test]
    fn typed_process_binding_accepts_exact_matching_reference_coordinates() {
        let process = ProcessDefinitionRefV1(manufacturing_ref(
            "ProcessDefinitionV1",
            SYMTHAEA_PROCESS_CRATE,
            'a',
        ));
        let binding = ProcessBindingV1 {
            process_definition: process,
            capability_requirement: ProcessCapabilityRequirementRefV1(manufacturing_ref(
                "ProcessCapabilityRequirementV1",
                SYMTHAEA_CONTRACTS_CRATE,
                'b',
            )),
            recipe: Some(ProcessRecipeRefV1(manufacturing_ref(
                "ProcessRecipeV1",
                SYMTHAEA_CONTRACTS_CRATE,
                'd',
            ))),
        };
        assert!(binding.validate().is_ok());
    }

    #[test]
    fn typed_machine_capability_ref_requires_profile_type_not_string_label() {
        let reference = MachineProcessCapabilityRefV1 {
            capability_profile: ProcessCapabilityProfileRefV1(manufacturing_ref(
                "ProcessCapabilityProfileV1",
                SYMTHAEA_CONTRACTS_CRATE,
                'e',
            )),
        };
        assert!(reference.validate().is_ok());
    }

    #[test]
    fn malformed_hex_and_wrong_contract_types_are_rejected() {
        let mut reference =
            manufacturing_ref("ProcessCapabilityProfileV1", SYMTHAEA_CONTRACTS_CRATE, 'e');
        reference.object_id_blake3 = "not-a-digest".into();
        assert!(ProcessCapabilityProfileRefV1(reference).validate().is_err());

        let wrong_type = ProcessCapabilityRequirementRefV1(manufacturing_ref(
            "ProcessRecipeV1",
            SYMTHAEA_CONTRACTS_CRATE,
            'b',
        ));
        assert!(wrong_type.validate().is_err());
    }

    #[test]
    fn legacy_machine_and_work_order_models_remain_unchanged() {
        let old_machine = Machine {
            name: "Legacy Mill".into(),
            machine_type: MachineType::CNC3Axis,
            capabilities: vec!["milling".into()],
            location: "Bay 1".into(),
            max_throughput_per_hour: 4,
            status: MachineStatus::Available,
            current_work_order: None,
            registered_at: test_timestamp(),
        };
        assert_eq!(old_machine.capabilities, vec!["milling".to_owned()]);
    }

    #[test]
    fn typed_process_migration_fixture_corpus_is_frozen_and_fail_closed() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/mfg_proc_001a_typed_process_refs_v0_1.json"
        ))
        .expect("typed process migration fixture JSON must parse");

        assert_eq!(
            fixture["schema_id"],
            "mycelix.manufacturing.mfg-proc-001a.typed-process-refs.v0_1"
        );
        let cases = fixture["cases"]
            .as_array()
            .expect("typed process fixture cases must be an array");
        assert_eq!(fixture["case_count"].as_u64(), Some(cases.len() as u64));
        assert_eq!(cases.len(), 16);

        for case in cases {
            assert!(
                case["id"]
                    .as_str()
                    .is_some_and(|id| id.starts_with("MFG-PROC-001A-")),
                "every frozen case has a stable scenario ID"
            );
            assert_eq!(
                case["may_satisfy_typed_matching"], false,
                "synthetic fixture vectors must not mint verified/qualified capability"
            );
        }

        let verified_case = cases
            .iter()
            .find(|case| case["expected_state"] == "OwnerObjectVerified")
            .expect("corpus retains an explicit verified-state negative control");
        assert_eq!(verified_case["reachable_in_v0_1"], false);
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
            feasible: false,
        };
        assert!(!result.feasible);
        assert_eq!(result.material_shortages[0].short_quantity, 80);
    }
}
