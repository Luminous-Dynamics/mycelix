//! Concrete refinement adapters over the canonical manufacturing_common types.
//!
//! These adapters intentionally stop at the strongest claim the source type can
//! justify. A source object's existence or lifecycle status never manufactures a
//! missing observation, consumption receipt, qualification, or temporal proof.

use manufacturing_common::{
    BillOfMaterials, Machine, MachineStatus, MrpResult, WorkOrder, WorkOrderStatus,
};
use crate::Decision;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceFact {
    WorkOrder,
    Bom,
    MrpPlan,
    MachineCapability,
    MachineStatus,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetFact {
    ProductionPlan,
    MaterialRequirement,
    ScheduledOperation,
    Capability,
    CurrentAvailability,
    ObservedWork,
    MaterialConsumption,
    QualifiedOutput,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceRefinement {
    pub source: SourceFact,
    pub target: TargetFact,
    pub decision: Decision,
    pub reason: &'static str,
    pub claim_ceiling: &'static str,
}

const SEMANTIC_CEILING: &str =
    "Source refinement only; no physical execution, safety, quality, or production-performance claim.";

pub fn work_order_to_production_plan(work_order: &WorkOrder) -> SourceRefinement {
    let decision = if work_order.product_id.is_empty() || work_order.quantity == 0 {
        Decision::Rejected
    } else {
        Decision::Accepted
    };
    SourceRefinement {
        source: SourceFact::WorkOrder,
        target: TargetFact::ProductionPlan,
        decision,
        reason: "A WorkOrder specifies intended production; it does not prove observed execution.",
        claim_ceiling: SEMANTIC_CEILING,
    }
}

pub fn completed_work_order_to_observed_work(work_order: &WorkOrder) -> SourceRefinement {
    let reason = match work_order.status {
        WorkOrderStatus::Completed | WorkOrderStatus::Closed =>
            "Completed/Closed is a lifecycle state, not an observed-work event or physical execution receipt.",
        _ => "A non-terminal WorkOrder status cannot prove observed work either.",
    };
    SourceRefinement {
        source: SourceFact::WorkOrder,
        target: TargetFact::ObservedWork,
        decision: Decision::Rejected,
        reason,
        claim_ceiling: SEMANTIC_CEILING,
    }
}

pub fn bom_to_material_requirement(bom: &BillOfMaterials) -> SourceRefinement {
    let decision = if bom.design_id.is_empty() || bom.revision.is_empty() || bom.items.is_empty() {
        Decision::Rejected
    } else {
        Decision::Accepted
    };
    SourceRefinement {
        source: SourceFact::Bom,
        target: TargetFact::MaterialRequirement,
        decision,
        reason: "BOM items establish planned material requirements, not actual consumption.",
        claim_ceiling: SEMANTIC_CEILING,
    }
}

pub fn bom_to_material_consumption(_bom: &BillOfMaterials) -> SourceRefinement {
    SourceRefinement {
        source: SourceFact::Bom,
        target: TargetFact::MaterialConsumption,
        decision: Decision::Rejected,
        reason: "BOM presence cannot create a consumption receipt, lot identity, quantity actually consumed, or timestamp.",
        claim_ceiling: SEMANTIC_CEILING,
    }
}

pub fn mrp_to_scheduled_operation(mrp: &MrpResult) -> SourceRefinement {
    let decision = if mrp.scheduled_operations.is_empty() {
        Decision::Rejected
    } else {
        Decision::Accepted
    };
    SourceRefinement {
        source: SourceFact::MrpPlan,
        target: TargetFact::ScheduledOperation,
        decision,
        reason: "MRP scheduling is planning evidence; it is not an execution observation.",
        claim_ceiling: SEMANTIC_CEILING,
    }
}

pub fn mrp_to_observed_work(_mrp: &MrpResult) -> SourceRefinement {
    SourceRefinement {
        source: SourceFact::MrpPlan,
        target: TargetFact::ObservedWork,
        decision: Decision::Rejected,
        reason: "A planned/scheduled operation does not prove that work occurred.",
        claim_ceiling: SEMANTIC_CEILING,
    }
}

pub fn machine_to_capability(machine: &Machine) -> SourceRefinement {
    let decision = if machine.capabilities.is_empty() {
        Decision::Rejected
    } else {
        Decision::Accepted
    };
    SourceRefinement {
        source: SourceFact::MachineCapability,
        target: TargetFact::Capability,
        decision,
        reason: "Registered machine capabilities support a capability claim at the registry layer.",
        claim_ceiling: SEMANTIC_CEILING,
    }
}

pub fn machine_status_to_current_availability(_machine: &Machine) -> SourceRefinement {
    SourceRefinement {
        source: SourceFact::MachineStatus,
        target: TargetFact::CurrentAvailability,
        decision: Decision::Rejected,
        reason: "MachineStatus has no observation timestamp, freshness policy, operator/evidence binding, or temporal validity interval.",
        claim_ceiling: SEMANTIC_CEILING,
    }
}

pub fn available_machine_status_to_observed_availability(machine: &Machine) -> SourceRefinement {
    let reason = if machine.status == MachineStatus::Available {
        "Available is a registry lifecycle value; an explicit timestamped availability observation is still required."
    } else {
        "Non-Available status cannot establish current availability."
    };
    SourceRefinement {
        source: SourceFact::MachineStatus,
        target: TargetFact::CurrentAvailability,
        decision: Decision::Rejected,
        reason,
        claim_ceiling: SEMANTIC_CEILING,
    }
}

pub fn work_order_to_qualified_output(_work_order: &WorkOrder) -> SourceRefinement {
    SourceRefinement {
        source: SourceFact::WorkOrder,
        target: TargetFact::QualifiedOutput,
        decision: Decision::Rejected,
        reason: "Work-order lifecycle completion does not establish quality, safety, acceptance criteria, or qualification evidence.",
        claim_ceiling: SEMANTIC_CEILING,
    }
}
