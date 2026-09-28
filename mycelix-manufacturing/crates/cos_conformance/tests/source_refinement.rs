use cos_conformance::source_refinement::{
    available_machine_status_to_observed_availability, bom_to_material_consumption,
    bom_to_material_requirement, completed_work_order_to_observed_work, machine_status_to_current_availability,
    machine_to_capability, mrp_to_observed_work, mrp_to_scheduled_operation,
    work_order_to_production_plan, work_order_to_qualified_output,
};
use cos_conformance::Decision;
use manufacturing_common::{
    BillOfMaterials, BomItem, Machine, MachineStatus, MachineType, MrpResult, Operation,
    RoutingSequence, RoutingStep, ScheduledOperation, WorkOrder, WorkOrderPriority,
    WorkOrderStatus,
};
use hdi::prelude::Timestamp;

fn ts() -> Timestamp {
    Timestamp::from_micros(1_700_000_000_000_000)
}

fn work_order(status: WorkOrderStatus) -> WorkOrder {
    WorkOrder {
        product_id: "PRODUCT-001".into(),
        quantity: 10,
        due_date: ts(),
        status,
        priority: WorkOrderPriority::Normal,
        notes: None,
        bom_hash: Some("bom-hash".into()),
        routing_hash: Some("routing-hash".into()),
        created_at: ts(),
        updated_at: ts(),
    }
}

fn bom() -> BillOfMaterials {
    BillOfMaterials {
        design_id: "DESIGN-001".into(),
        revision: "A".into(),
        items: vec![BomItem {
            part_id: "PART-001".into(),
            quantity_per: 2,
            unit: "each".into(),
            sub_assembly_bom_hash: None,
            notes: None,
        }],
        created_at: ts(),
    }
}

fn machine(status: MachineStatus) -> Machine {
    Machine {
        name: "CNC-1".into(),
        machine_type: MachineType::CNC3Axis,
        capabilities: vec!["milling".into()],
        location: "Bay-1".into(),
        max_throughput_per_hour: 10,
        status,
        current_work_order: None,
        registered_at: ts(),
    }
}

fn mrp() -> MrpResult {
    MrpResult {
        planned_orders: vec![],
        scheduled_operations: vec![ScheduledOperation {
            work_order_id: "WO-001".into(),
            operation_name: "Mill".into(),
            machine_id: "CNC-1".into(),
            start_time: ts(),
            end_time: ts(),
            setup_minutes: 5,
            run_minutes: 20,
        }],
        capacity_warnings: vec![],
        material_shortages: vec![],
        feasible: true,
    }
}

#[test]
fn completed_work_order_does_not_become_observed_work() {
    let result = completed_work_order_to_observed_work(&work_order(WorkOrderStatus::Completed));
    assert_eq!(result.decision, Decision::Rejected);
    assert!(result.reason.contains("lifecycle state"));
}

#[test]
fn closed_work_order_does_not_become_observed_work() {
    let result = completed_work_order_to_observed_work(&work_order(WorkOrderStatus::Closed));
    assert_eq!(result.decision, Decision::Rejected);
}

#[test]
fn work_order_can_refine_to_plan_but_not_qualified_output() {
    assert_eq!(
        work_order_to_production_plan(&work_order(WorkOrderStatus::Draft)).decision,
        Decision::Accepted
    );
    assert_eq!(
        work_order_to_qualified_output(&work_order(WorkOrderStatus::Completed)).decision,
        Decision::Rejected
    );
}

#[test]
fn bom_is_requirement_evidence_not_consumption_evidence() {
    assert_eq!(bom_to_material_requirement(&bom()).decision, Decision::Accepted);
    assert_eq!(bom_to_material_consumption(&bom()).decision, Decision::Rejected);
}

#[test]
fn mrp_schedule_is_not_observed_work() {
    assert_eq!(mrp_to_scheduled_operation(&mrp()).decision, Decision::Accepted);
    assert_eq!(mrp_to_observed_work(&mrp()).decision, Decision::Rejected);
}

#[test]
fn machine_capability_is_not_current_availability() {
    let available = machine(MachineStatus::Available);
    assert_eq!(machine_to_capability(&available).decision, Decision::Accepted);
    assert_eq!(machine_status_to_current_availability(&available).decision, Decision::Rejected);
    assert_eq!(available_machine_status_to_observed_availability(&available).decision, Decision::Rejected);
}

#[test]
fn machine_non_available_state_cannot_create_availability_evidence() {
    for status in [
        MachineStatus::Running,
        MachineStatus::Maintenance,
        MachineStatus::Offline,
    ] {
        assert_eq!(
            available_machine_status_to_observed_availability(&machine(status)).decision,
            Decision::Rejected
        );
    }
}
