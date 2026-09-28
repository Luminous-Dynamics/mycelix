//! Reference adapters for Integral's publicly documented Phase-0/DOC-02 data contracts.
//!
//! Integral's public specifications page labels these core data structures DRAFT
//! and the interface contracts PENDING, so these records are not treated as ratified APIs.

use crate::{Decision, Evidence, Origin};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DraftCertifiedDesign {
    pub design_id: String, pub version: String, pub bill_of_materials: String,
    pub production_steps: String, pub ecological_flag: bool, pub itc_access_cost: String,
    pub design_lineage: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DraftLaborEvent {
    pub event_id: String, pub timestamp: u64, pub participant_id: String, pub coop_id: String,
    pub task_ref: String, pub production_plan_ref: String, pub design_ref: String,
    pub hours: u64, pub hours_verified: u64, pub skill_tier: String, pub context: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DraftMaterialConsumptionEvent {
    pub event_id: String, pub timestamp: u64, pub material_id: String, pub quantity_consumed: u64,
    pub ecological_flag: bool, pub source: String, pub production_ref: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceBoundLabor {
    pub event_id: String, pub participant_id: String, pub coop_id: String, pub task_ref: String,
    pub production_plan_ref: String, pub design_ref: String, pub hours_verified: u64,
    pub skill_tier: String, pub source_evidence_id: String, pub origin: Origin,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceBoundMaterialConsumption {
    pub event_id: String, pub material_id: String, pub quantity_consumed: u64,
    pub source: String, pub production_ref: String, pub source_evidence_id: String, pub origin: Origin,
}

pub fn certified_design_is_reference_only(_design: &DraftCertifiedDesign) -> bool { true }

pub fn bind_verified_labor(event: &DraftLaborEvent, evidence: Evidence) -> Result<SourceBoundLabor, Decision> {
    if event.event_id.is_empty() || event.participant_id.is_empty() || event.task_ref.is_empty()
        || event.production_plan_ref.is_empty() || event.design_ref.is_empty()
        || event.hours == 0 || event.hours_verified == 0 || event.hours_verified > event.hours {
        return Err(Decision::Unbound);
    }
    if !evidence.validity.active_at(event.timestamp) { return Err(Decision::Stale); }
    Ok(SourceBoundLabor {
        event_id: event.event_id.clone(), participant_id: event.participant_id.clone(),
        coop_id: event.coop_id.clone(), task_ref: event.task_ref.clone(),
        production_plan_ref: event.production_plan_ref.clone(), design_ref: event.design_ref.clone(),
        hours_verified: event.hours_verified, skill_tier: event.skill_tier.clone(),
        source_evidence_id: evidence.id.clone(), origin: evidence.origin.clone(),
    })
}

pub fn bind_material_consumption(event: &DraftMaterialConsumptionEvent, evidence: Evidence)
    -> Result<SourceBoundMaterialConsumption, Decision> {
    if event.event_id.is_empty() || event.material_id.is_empty() || event.quantity_consumed == 0
        || event.production_ref.is_empty() {
        return Err(Decision::Unbound);
    }
    if !evidence.validity.active_at(event.timestamp) { return Err(Decision::Stale); }
    Ok(SourceBoundMaterialConsumption {
        event_id: event.event_id.clone(), material_id: event.material_id.clone(),
        quantity_consumed: event.quantity_consumed, source: event.source.clone(),
        production_ref: event.production_ref.clone(), source_evidence_id: evidence.id.clone(),
        origin: evidence.origin.clone(),
    })
}

pub fn source_observation_grants_ledger_authority() -> bool { false }

#[cfg(test)]
mod tests {
    use super::*;
    fn labor() -> DraftLaborEvent {
        DraftLaborEvent {
            event_id: "le-1".into(), timestamp: 100, participant_id: "p-1".into(),
            coop_id: "coop-1".into(), task_ref: "task-1".into(), production_plan_ref: "plan-1".into(),
            design_ref: "design-1".into(), hours: 5, hours_verified: 4,
            skill_tier: "medium".into(), context: "ctx".into(),
        }
    }
    fn material() -> DraftMaterialConsumptionEvent {
        DraftMaterialConsumptionEvent {
            event_id: "mc-1".into(), timestamp: 100, material_id: "mat-1".into(),
            quantity_consumed: 7, ecological_flag: true, source: "foreign-supplier".into(),
            production_ref: "plan-1".into(),
        }
    }
    #[test] fn verified_hours_are_preserved_not_reconstructed() {
        let bound = bind_verified_labor(&labor(), Evidence::current_local("ev-1", 100)).unwrap();
        assert_eq!(bound.hours_verified, 4); assert_eq!(bound.event_id, "le-1");
    }
    #[test] fn claimed_hours_without_verification_are_rejected() {
        let mut e = labor(); e.hours_verified = 0;
        assert_eq!(bind_verified_labor(&e, Evidence::current_local("ev-1", 100)), Err(Decision::Unbound));
    }
    #[test] fn over_verified_hours_are_rejected() {
        let mut e = labor(); e.hours_verified = 6;
        assert_eq!(bind_verified_labor(&e, Evidence::current_local("ev-1", 100)), Err(Decision::Unbound));
    }
    #[test] fn stale_labor_evidence_is_rejected() {
        assert_eq!(bind_verified_labor(&labor(), Evidence::stale("ev-stale", 100)), Err(Decision::Stale));
    }
    #[test] fn material_quantity_is_source_bound() {
        let bound = bind_material_consumption(&material(), Evidence::current_foreign("ev-mat", "node-b", 100)).unwrap();
        assert_eq!(bound.quantity_consumed, 7);
        assert_eq!(bound.origin, Origin::Foreign("node-b".into()));
    }
    #[test] fn zero_consumption_is_not_a_consumption_event() {
        let mut e = material(); e.quantity_consumed = 0;
        assert_eq!(bind_material_consumption(&e, Evidence::current_local("ev", 100)), Err(Decision::Unbound));
    }
    #[test] fn draft_certified_design_does_not_mint_authority() {
        let d = DraftCertifiedDesign {
            design_id: "d-1".into(), version: "1".into(), bill_of_materials: "bom".into(),
            production_steps: "steps".into(), ecological_flag: false, itc_access_cost: "cost".into(),
            design_lineage: None,
        };
        assert!(certified_design_is_reference_only(&d));
        assert!(!source_observation_grants_ledger_authority());
    }
}
