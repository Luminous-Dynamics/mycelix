//! Source-owned COS observations projected into ITC/FRS without semantic collapse.

use crate::{Decision, Evidence, Origin};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservationKind { Labor, MaterialConsumption, QualityOutcome, Throughput }

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CosObservation {
    pub id: String,
    pub kind: ObservationKind,
    pub source_evidence: Evidence,
    pub observed_at: u64,
    pub verified: bool,
    pub participant_ref: Option<String>,
    pub labor_hours: Option<u64>,
    pub material_ref: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ItcProjection {
    pub source_observation_id: String,
    pub participant_ref: Option<String>,
    pub labor_hours: Option<u64>,
    pub material_ref: Option<String>,
    pub projection_generation: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FrsSignal {
    pub source_observation_ids: Vec<String>,
    pub signal_generation: u32,
    pub interpretation: String,
}

fn admissible(observation: &CosObservation) -> Result<(), Decision> {
    if !observation.verified { return Err(Decision::Unbound); }
    if !observation.source_evidence.validity.active_at(observation.observed_at) {
        return Err(Decision::Stale);
    }
    Ok(())
}

pub fn project_to_itc(observation: &CosObservation, generation: u32) -> Result<ItcProjection, Decision> {
    admissible(observation)?;
    match observation.kind {
        ObservationKind::Labor | ObservationKind::MaterialConsumption => Ok(ItcProjection {
            source_observation_id: observation.id.clone(),
            participant_ref: observation.participant_ref.clone(),
            labor_hours: observation.labor_hours,
            material_ref: observation.material_ref.clone(),
            projection_generation: generation,
        }),
        ObservationKind::QualityOutcome | ObservationKind::Throughput => Err(Decision::Rejected),
    }
}

pub fn project_to_frs(observation: &CosObservation, generation: u32) -> Result<FrsSignal, Decision> {
    admissible(observation)?;
    Ok(FrsSignal {
        source_observation_ids: vec![observation.id.clone()],
        signal_generation: generation,
        interpretation: "Derived signal; not a replacement source observation.".into(),
    })
}

pub fn projection_preserves_origin(observation: &CosObservation) -> Origin {
    observation.source_evidence.origin.clone()
}

pub fn projection_grants_authority() -> bool { false }

#[cfg(test)]
mod tests {
    use super::*;

    fn labor() -> CosObservation {
        CosObservation {
            id: "cos-labor-1".into(),
            kind: ObservationKind::Labor,
            source_evidence: Evidence::current_local("labor-evidence-1", 100),
            observed_at: 100,
            verified: true,
            participant_ref: Some("participant-1".into()),
            labor_hours: Some(4),
            material_ref: None,
        }
    }

    #[test]
    fn verified_labor_projects_exact_source_values_to_itc() {
        let projection = project_to_itc(&labor(), 1).unwrap();
        assert_eq!(projection.source_observation_id, "cos-labor-1");
        assert_eq!(projection.participant_ref.as_deref(), Some("participant-1"));
        assert_eq!(projection.labor_hours, Some(4));
        assert!(!projection_grants_authority());
    }

    #[test]
    fn unverified_labor_cannot_project_to_itc() {
        let mut observation = labor(); observation.verified = false;
        assert_eq!(project_to_itc(&observation, 1), Err(Decision::Unbound));
    }

    #[test]
    fn stale_observation_cannot_project_to_itc_or_frs() {
        let observation = CosObservation {
            source_evidence: Evidence::stale("stale-1", 100),
            ..labor()
        };
        assert_eq!(project_to_itc(&observation, 1), Err(Decision::Stale));
        assert_eq!(project_to_frs(&observation, 1), Err(Decision::Stale));
    }

    #[test]
    fn material_consumption_projects_but_quality_and_throughput_do_not_mint_itc() {
        let material = CosObservation {
            kind: ObservationKind::MaterialConsumption,
            material_ref: Some("material-1".into()),
            labor_hours: None,
            participant_ref: None,
            ..labor()
        };
        assert!(project_to_itc(&material, 1).is_ok());
        for kind in [ObservationKind::QualityOutcome, ObservationKind::Throughput] {
            let observation = CosObservation { kind, ..labor() };
            assert_eq!(project_to_itc(&observation, 1), Err(Decision::Rejected));
        }
    }

    #[test]
    fn frs_projection_retains_source_identity() {
        let signal = project_to_frs(&labor(), 4).unwrap();
        assert_eq!(signal.source_observation_ids, vec!["cos-labor-1"]);
        assert!(signal.interpretation.contains("not a replacement"));
    }

    #[test]
    fn foreign_origin_survives_projection() {
        let mut observation = labor();
        observation.source_evidence = Evidence::current_foreign("foreign-labor", "node-b", 100);
        let _ = project_to_itc(&observation, 1).unwrap();
        let _ = project_to_frs(&observation, 1).unwrap();
        assert_eq!(projection_preserves_origin(&observation), Origin::Foreign("node-b".into()));
    }
}
