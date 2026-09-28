use cos_conformance::integral_phase0_schema::{
    bind_material_consumption, bind_verified_labor, DraftLaborEvent, DraftMaterialConsumptionEvent,
};
use cos_conformance::{Decision, Evidence, Origin};

fn labor() -> DraftLaborEvent {
    DraftLaborEvent {
        event_id: "labor-1".into(), timestamp: 100, participant_id: "p-1".into(),
        coop_id: "coop-1".into(), task_ref: "task-1".into(), production_plan_ref: "plan-1".into(),
        design_ref: "design-1".into(), hours: 8, hours_verified: 6,
        skill_tier: "high".into(), context: "ctx".into(),
    }
}

#[test]
fn public_draft_labor_schema_becomes_source_bound_evidence() {
    let x = bind_verified_labor(&labor(), Evidence::current_local("ev-1", 100)).unwrap();
    assert_eq!(x.hours_verified, 6); assert_eq!(x.production_plan_ref, "plan-1");
}

#[test]
fn unverified_or_stale_labor_never_enters_projection_boundary() {
    let mut x = labor(); x.hours_verified = 0;
    assert_eq!(bind_verified_labor(&x, Evidence::current_local("ev-1", 100)), Err(Decision::Unbound));
    assert_eq!(bind_verified_labor(&labor(), Evidence::stale("ev-2", 100)), Err(Decision::Stale));
}

#[test]
fn foreign_material_origin_is_retained() {
    let x = DraftMaterialConsumptionEvent {
        event_id: "mat-1".into(), timestamp: 100, material_id: "m-1".into(), quantity_consumed: 3,
        ecological_flag: false, source: "node-b".into(), production_ref: "plan-1".into(),
    };
    let bound = bind_material_consumption(&x, Evidence::current_foreign("ev", "node-b", 100)).unwrap();
    assert_eq!(bound.origin, Origin::Foreign("node-b".into()));
}
