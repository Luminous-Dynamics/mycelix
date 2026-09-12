// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance envelope for multi-epoch regenerative-lineage experiments.
//!
//! This composes existing Mycelix handoff + viability provenance with recovery-aware
//! role traces and opaque calibration evidence. It validates record agreement and
//! lineage continuity; it does not recompute Symtropy dynamics or Symthaea models.

use crate::{
    verify_regenerative_epoch_handoff_provenance, MaritimeEvidenceEnvelope, MaritimeEvidenceKind,
    RegenerativeEpochHandoffProvenanceV1, RegenerativeGenomeLineageEvidenceV1,
    RegenerativeViabilityEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_LINEAGE_EXPERIMENT_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_ROLES: usize = 64;
const MAX_TRANSITION_REFS: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegenerativeLineageExperimentOutcomeV1 {
    SuccessorReproductionPreservedThroughObservation,
    LineageReproductionLostBeforeOperation,
    OperationLostBeforeReproductionAssessment,
    Inconclusive,
}

/// Recovery-aware provenance summary for one role in a dynamic lineage experiment.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeLineageExperimentRoleTraceV1 {
    pub role_id: String,
    pub first_unavailable_tick: Option<u64>,
    pub first_recovery_tick: Option<u64>,
    pub available_at_end: bool,
    /// Opaque transition/event references, strictly sorted and unique.
    pub transition_evidence_refs: Vec<String>,
}

impl RegenerativeLineageExperimentRoleTraceV1 {
    fn validate(&self, observed_through_tick: u64) -> Result<(), String> {
        if !canonical_id(&self.role_id) {
            return Err("experiment role_id is not canonical".into());
        }
        if let Some(first_unavailable) = self.first_unavailable_tick {
            if first_unavailable == 0 || first_unavailable > observed_through_tick {
                return Err("role first_unavailable_tick is outside the observation window".into());
            }
        } else if !self.available_at_end {
            return Err("role cannot end unavailable without an observed first-unavailable tick".into());
        }
        if let Some(first_recovery) = self.first_recovery_tick {
            let first_unavailable = self
                .first_unavailable_tick
                .ok_or_else(|| "role recovery requires a prior unavailable observation".to_string())?;
            if first_recovery <= first_unavailable || first_recovery > observed_through_tick {
                return Err("role first_recovery_tick is outside the valid recovery window".into());
            }
        }
        if self.available_at_end
            && self.first_unavailable_tick.is_some()
            && self.first_recovery_tick.is_none()
        {
            return Err("role ending available after failure requires recovery evidence".into());
        }
        if self.transition_evidence_refs.len() > MAX_TRANSITION_REFS {
            return Err(format!(
                "too many role transition references (max {MAX_TRANSITION_REFS})"
            ));
        }
        for reference in &self.transition_evidence_refs {
            if !canonical_reference(reference) {
                return Err("role transition evidence reference is not canonical".into());
            }
        }
        if self
            .transition_evidence_refs
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
        {
            return Err("role transition evidence refs must be sorted and duplicate-free".into());
        }
        Ok(())
    }
}

/// Strict provenance for one complete evolving-lineage experiment interpretation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeLineageExperimentEvidenceV1 {
    pub schema_version: u8,
    pub experiment_id: String,
    /// Raw content digest of the exact Mycelix epoch-handoff provenance record.
    pub handoff_provenance_content_digest: String,
    /// Raw content digest of the exact Mycelix viability provenance record.
    pub viability_evidence_content_digest: String,
    /// Exact dynamic run/subject identity. Must match viability provenance.
    pub symtropy_run_binding: String,
    /// Exact upstream Symthaea calibration/report identity.
    pub symthaea_calibration_binding: String,
    pub observed_through_tick: u64,
    /// First tick of the adaptation phase, if the scenario declares one.
    pub adaptation_started_tick: Option<u64>,
    /// Strictly sorted by role_id.
    pub roles: Vec<RegenerativeLineageExperimentRoleTraceV1>,
    /// Upstream dynamic claim: whether a completed post-adaptation observation had
    /// both construction and qualification available simultaneously.
    pub successor_reproduction_observed_after_adaptation: bool,
    /// Upstream calibration claim preserved without recomputation by Mycelix.
    pub static_model_conflict: bool,
    /// Roles identified upstream as model-refinement opportunities, sorted/unique.
    pub refinement_role_ids: Vec<String>,
    pub outcome: RegenerativeLineageExperimentOutcomeV1,
}

impl RegenerativeLineageExperimentEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_LINEAGE_EXPERIMENT_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative lineage experiment schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.experiment_id) {
            return Err("experiment_id is not canonical".into());
        }
        if !lower_hex_64(&self.handoff_provenance_content_digest)
            || !lower_hex_64(&self.viability_evidence_content_digest)
        {
            return Err("composed provenance digests must be canonical lowercase 64-hex".into());
        }
        for binding in [
            &self.symtropy_run_binding,
            &self.symthaea_calibration_binding,
        ] {
            if !canonical_reference(binding) {
                return Err("experiment upstream evidence binding is not canonical".into());
            }
        }
        if self.observed_through_tick == 0 {
            return Err("experiment must include at least one completed observed tick".into());
        }
        if self
            .adaptation_started_tick
            .is_some_and(|tick| tick == 0 || tick > self.observed_through_tick)
        {
            return Err("adaptation_started_tick is outside the observation window".into());
        }
        if self.roles.is_empty() || self.roles.len() > MAX_ROLES {
            return Err(format!("experiment requires 1..={MAX_ROLES} role traces"));
        }
        for role in &self.roles {
            role.validate(self.observed_through_tick)?;
        }
        if self
            .roles
            .windows(2)
            .any(|pair| pair[0].role_id >= pair[1].role_id)
        {
            return Err("experiment roles must be strictly sorted and duplicate-free".into());
        }
        if self.refinement_role_ids.len() > MAX_ROLES {
            return Err(format!("too many refinement roles (max {MAX_ROLES})"));
        }
        for role_id in &self.refinement_role_ids {
            if !canonical_id(role_id)
                || !self.roles.iter().any(|role| &role.role_id == role_id)
            {
                return Err("refinement role must name a role carried by the experiment".into());
            }
        }
        if self
            .refinement_role_ids
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
        {
            return Err("refinement roles must be strictly sorted and duplicate-free".into());
        }
        self.validate_outcome_consistency()
    }

    fn validate_outcome_consistency(&self) -> Result<(), String> {
        let operation = self.roles.iter().find(|role| role.role_id == "operation");
        let construction = self
            .roles
            .iter()
            .find(|role| role.role_id == "successor_construction");
        let qualification = self
            .roles
            .iter()
            .find(|role| role.role_id == "successor_qualification");

        match self.outcome {
            RegenerativeLineageExperimentOutcomeV1::SuccessorReproductionPreservedThroughObservation => {
                if !self.successor_reproduction_observed_after_adaptation {
                    return Err("preserved-reproduction outcome requires observed reproduction availability".into());
                }
            }
            RegenerativeLineageExperimentOutcomeV1::LineageReproductionLostBeforeOperation => {
                if self.successor_reproduction_observed_after_adaptation {
                    return Err("lost-reproduction outcome contradicts observed reproduction availability".into());
                }
                let operation_failure = operation
                    .and_then(|role| role.first_unavailable_tick)
                    .ok_or_else(|| "lost-before-operation outcome requires operation failure evidence".to_string())?;
                let reproduction_failed_earlier = [construction, qualification]
                    .into_iter()
                    .flatten()
                    .any(|role| {
                        role.first_unavailable_tick
                            .is_some_and(|tick| tick < operation_failure)
                    });
                if !reproduction_failed_earlier {
                    return Err("lost-before-operation outcome requires earlier construction/qualification loss".into());
                }
            }
            RegenerativeLineageExperimentOutcomeV1::OperationLostBeforeReproductionAssessment => {
                if operation.and_then(|role| role.first_unavailable_tick).is_none() {
                    return Err("operation-lost outcome requires operation failure evidence".into());
                }
            }
            RegenerativeLineageExperimentOutcomeV1::Inconclusive => {}
        }
        Ok(())
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize lineage experiment evidence: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-lineage-experiment-v1\0");
        hasher.update(&(payload.len() as u64).to_le_bytes());
        hasher.update(payload.as_bytes());
        Ok(hasher.finalize().to_hex().to_string())
    }

    pub fn to_maritime_envelope(
        &self,
        platform_id: impl Into<String>,
        generation: u64,
        sequence: u64,
        observed_at_us: u64,
        event_evidence_binding: impl Into<String>,
    ) -> Result<MaritimeEvidenceEnvelope, String> {
        let envelope = MaritimeEvidenceEnvelope::new(
            platform_id,
            generation,
            sequence,
            observed_at_us,
            MaritimeEvidenceKind::LogisticsEvent,
            self.to_payload_json()?,
            event_evidence_binding,
        );
        envelope.validate()?;
        Ok(envelope)
    }
}

/// Verify agreement between the composed Mycelix provenance records and one
/// recovery-aware experiment envelope.
pub fn verify_regenerative_lineage_experiment_evidence(
    source_lineage: &RegenerativeGenomeLineageEvidenceV1,
    successor_lineage: &RegenerativeGenomeLineageEvidenceV1,
    handoff: &RegenerativeEpochHandoffProvenanceV1,
    viability: &RegenerativeViabilityEvidenceV1,
    experiment: &RegenerativeLineageExperimentEvidenceV1,
) -> Result<(), String> {
    experiment.validate()?;
    verify_regenerative_epoch_handoff_provenance(
        source_lineage,
        successor_lineage,
        handoff,
    )?;
    viability.validate()?;

    if experiment.handoff_provenance_content_digest != handoff.content_digest()? {
        return Err("experiment handoff provenance digest mismatch".into());
    }
    if experiment.viability_evidence_content_digest != viability.content_digest()? {
        return Err("experiment viability provenance digest mismatch".into());
    }
    if viability.genome_binding != handoff.successor_genome_binding
        || viability.genome_binding != successor_lineage.genome_binding
    {
        return Err("experiment successor Genome binding is inconsistent across provenance".into());
    }
    let expected_lineage_binding = format!(
        "lineage:blake3:{}",
        successor_lineage.content_digest()?
    );
    if viability.lineage_evidence_binding != expected_lineage_binding {
        return Err("viability evidence does not bind the exact successor lineage record".into());
    }
    if experiment.symtropy_run_binding != viability.dynamic_simulation_binding {
        return Err("experiment dynamic run binding disagrees with viability evidence".into());
    }
    if experiment.roles.len() != viability.roles.len() {
        return Err("experiment role coverage differs from viability evidence".into());
    }
    for (trace, role) in experiment.roles.iter().zip(&viability.roles) {
        if trace.role_id != role.role_id
            || trace.first_unavailable_tick != role.dynamic_first_unavailable_tick
        {
            return Err("experiment role trace disagrees with viability first-failure evidence".into());
        }
    }
    Ok(())
}

fn canonical_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_ID_BYTES
        && value.trim() == value
        && !value.chars().any(char::is_whitespace)
        && !value.chars().any(char::is_control)
}

fn canonical_reference(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_BINDING_BYTES
        && value.trim() == value
        && value.contains(':')
        && !value.chars().any(char::is_whitespace)
        && !value.chars().any(char::is_control)
}

fn lower_hex_64(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        RegenerativeEpochExternalAdmissionProvenanceV1, RegenerativeEpochTransferProvenanceV1,
        RegenerativeViabilityHorizonV1, RegenerativeViabilityRoleEvidenceV1,
        REGENERATIVE_EPOCH_HANDOFF_PROVENANCE_SCHEMA_V1,
        REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1, REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1,
    };

    const SYMTROPY_HEAD: &str = "10e2bb188d7cb043b0ae74d11b8718db103268dd";
    const SYMTHAEA_HEAD: &str = "d15681da30d99ec22c905a6e1e435f6e4cf22504";

    fn source_lineage() -> RegenerativeGenomeLineageEvidenceV1 {
        RegenerativeGenomeLineageEvidenceV1 {
            schema_version: REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1,
            genome_id: "genome-v1".into(),
            genome_binding: "genome:manta-v1-lineage-experiment".into(),
            parent_genome_binding: None,
            closure_model_binding: "model:manta-v1-lineage-experiment".into(),
            supportability_report_binding: "supportability:manta-v1-lineage-experiment".into(),
            qualification_binding: "qualification:manta-v1-lineage-experiment".into(),
            requirement_evidence_refs: vec!["requirement:operation-v1".into()],
            substitution_evidence_refs: Vec::new(),
        }
    }

    fn successor_lineage() -> RegenerativeGenomeLineageEvidenceV1 {
        RegenerativeGenomeLineageEvidenceV1 {
            schema_version: REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1,
            genome_id: "genome-v2".into(),
            genome_binding: "genome:manta-v2-lineage-experiment".into(),
            parent_genome_binding: Some("genome:manta-v1-lineage-experiment".into()),
            closure_model_binding: "model:manta-v2-lineage-experiment".into(),
            supportability_report_binding: "supportability:manta-v2-lineage-experiment".into(),
            qualification_binding: "qualification:manta-v2-lineage-experiment".into(),
            requirement_evidence_refs: vec!["requirement:operation-v2".into()],
            substitution_evidence_refs: vec!["substitution:local-controller-v2".into()],
        }
    }

    fn handoff() -> RegenerativeEpochHandoffProvenanceV1 {
        let source = source_lineage();
        let successor = successor_lineage();
        RegenerativeEpochHandoffProvenanceV1 {
            schema_version: REGENERATIVE_EPOCH_HANDOFF_PROVENANCE_SCHEMA_V1,
            handoff_id: "handoff-manta-v1-v2-lineage-experiment".into(),
            source_epoch_binding: "epoch:manta-v1-lineage-experiment".into(),
            successor_epoch_binding: "epoch:manta-v2-qualified-substitution".into(),
            source_genome_binding: source.genome_binding.clone(),
            successor_genome_binding: successor.genome_binding.clone(),
            source_lineage_content_digest: source.content_digest().unwrap(),
            successor_lineage_content_digest: successor.content_digest().unwrap(),
            source_closure_model_binding: source.closure_model_binding.clone(),
            successor_closure_model_binding: successor.closure_model_binding.clone(),
            symthaea_handoff_qualification_binding: "symthaea-pr:1800:fc4d2f8e0a50885822549f574fe66b8c9de302ff".into(),
            symtropy_handoff_receipt_binding: "symtropy-pr:751:d2c1c954d3bd89eeb2ca8f5463f40363f8ba2651".into(),
            cross_repo_fixture_binding:
                "git-blob:725aa1eadf417eae1be95829b0e93fe7e9c9ee60".into(),
            transfers: vec![
                RegenerativeEpochTransferProvenanceV1 {
                    source_dependency_id: "forge-tooling-v1".into(),
                    successor_dependency_id: "forge-tooling-v2".into(),
                    transferred_units: 6,
                    retired_units: 0,
                    transfer_qualification_binding: "qualification:forge-tooling-v1-v2".into(),
                    safeguarded_continuity_binding: None,
                },
                RegenerativeEpochTransferProvenanceV1 {
                    source_dependency_id: "metrology-v1".into(),
                    successor_dependency_id: "metrology-v2".into(),
                    transferred_units: 4,
                    retired_units: 0,
                    transfer_qualification_binding: "qualification:metrology-v1-v2".into(),
                    safeguarded_continuity_binding: None,
                },
                RegenerativeEpochTransferProvenanceV1 {
                    source_dependency_id: "reactor-service-v1".into(),
                    successor_dependency_id: "reactor-service-v2".into(),
                    transferred_units: 8,
                    retired_units: 0,
                    transfer_qualification_binding: "qualification:reactor-service-v1-v2".into(),
                    safeguarded_continuity_binding: Some(
                        "safeguarded:reactor-service-v1-v2".into(),
                    ),
                },
                RegenerativeEpochTransferProvenanceV1 {
                    source_dependency_id: "structural-stock-v1".into(),
                    successor_dependency_id: "structural-stock-v2".into(),
                    transferred_units: 18,
                    retired_units: 0,
                    transfer_qualification_binding: "qualification:structural-stock-v1-v2".into(),
                    safeguarded_continuity_binding: None,
                },
            ],
            external_admissions: Vec::<RegenerativeEpochExternalAdmissionProvenanceV1>::new(),
        }
    }

    fn viability() -> RegenerativeViabilityEvidenceV1 {
        let successor = successor_lineage();
        RegenerativeViabilityEvidenceV1 {
            schema_version: REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1,
            genome_binding: successor.genome_binding.clone(),
            lineage_evidence_binding: format!(
                "lineage:blake3:{}",
                successor.content_digest().unwrap()
            ),
            viability_profile_binding: "viability-profile:manta-v2-lineage-experiment".into(),
            static_viability_report_binding: format!("symthaea-pr:1816:{SYMTHAEA_HEAD}"),
            dynamic_simulation_binding: format!("symtropy-pr:766:{SYMTROPY_HEAD}"),
            closure_model_binding: successor.closure_model_binding.clone(),
            flow_support_binding: "flow-support:manta-v2-lineage-experiment".into(),
            period_duration_ms: 1,
            roles: vec![
                RegenerativeViabilityRoleEvidenceV1 {
                    role_id: "operation".into(),
                    static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(6),
                    dynamic_first_unavailable_tick: Some(9),
                    fully_modeled_support: true,
                    limiting_dependency_refs: vec!["dependency:forge-tooling-v2".into()],
                },
                RegenerativeViabilityRoleEvidenceV1 {
                    role_id: "successor_construction".into(),
                    static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(6),
                    dynamic_first_unavailable_tick: Some(6),
                    fully_modeled_support: true,
                    limiting_dependency_refs: vec!["dependency:forge-tooling-v2".into()],
                },
                RegenerativeViabilityRoleEvidenceV1 {
                    role_id: "successor_qualification".into(),
                    static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(8),
                    dynamic_first_unavailable_tick: Some(5),
                    fully_modeled_support: true,
                    limiting_dependency_refs: vec!["dependency:reactor-service-v2".into()],
                },
            ],
            regenerative_viability_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(6),
            limiting_role_ids: vec!["operation".into(), "successor_construction".into()],
            fully_modeled_regenerative_viability: true,
        }
    }

    fn experiment() -> RegenerativeLineageExperimentEvidenceV1 {
        let handoff = handoff();
        let viability = viability();
        RegenerativeLineageExperimentEvidenceV1 {
            schema_version: REGENERATIVE_LINEAGE_EXPERIMENT_SCHEMA_V1,
            experiment_id: "manta-forge-lineage-experiment-v1".into(),
            handoff_provenance_content_digest: handoff.content_digest().unwrap(),
            viability_evidence_content_digest: viability.content_digest().unwrap(),
            symtropy_run_binding: format!("symtropy-pr:766:{SYMTROPY_HEAD}"),
            symthaea_calibration_binding: format!("symthaea-pr:1816:{SYMTHAEA_HEAD}"),
            observed_through_tick: 9,
            adaptation_started_tick: Some(6),
            roles: vec![
                RegenerativeLineageExperimentRoleTraceV1 {
                    role_id: "operation".into(),
                    first_unavailable_tick: Some(9),
                    first_recovery_tick: None,
                    available_at_end: false,
                    transition_evidence_refs: vec!["transition:operation:tick9".into()],
                },
                RegenerativeLineageExperimentRoleTraceV1 {
                    role_id: "successor_construction".into(),
                    first_unavailable_tick: Some(6),
                    first_recovery_tick: None,
                    available_at_end: false,
                    transition_evidence_refs: vec![
                        "transition:construction-controller:tick6".into(),
                        "transition:construction-tooling:tick7".into(),
                    ],
                },
                RegenerativeLineageExperimentRoleTraceV1 {
                    role_id: "successor_qualification".into(),
                    first_unavailable_tick: Some(5),
                    first_recovery_tick: Some(6),
                    available_at_end: false,
                    transition_evidence_refs: vec![
                        "transition:qualification:tick5-unavailable".into(),
                        "transition:qualification:tick6-recovered".into(),
                        "transition:qualification:tick9-unavailable".into(),
                    ],
                },
            ],
            successor_reproduction_observed_after_adaptation: false,
            static_model_conflict: false,
            refinement_role_ids: vec!["operation".into()],
            outcome: RegenerativeLineageExperimentOutcomeV1::LineageReproductionLostBeforeOperation,
        }
    }

    #[test]
    fn exact_experiment_composes_handoff_viability_and_recovery_evidence() {
        let source = source_lineage();
        let successor = successor_lineage();
        let handoff = handoff();
        let viability = viability();
        let experiment = experiment();
        assert_eq!(
            verify_regenerative_lineage_experiment_evidence(
                &source,
                &successor,
                &handoff,
                &viability,
                &experiment,
            ),
            Ok(())
        );
        assert_eq!(
            experiment.outcome,
            RegenerativeLineageExperimentOutcomeV1::LineageReproductionLostBeforeOperation
        );
        assert!(!experiment.static_model_conflict);
        assert_eq!(experiment.refinement_role_ids, vec!["operation"]);
    }

    #[test]
    fn tampered_dynamic_run_or_first_failure_fails_composition() {
        let source = source_lineage();
        let successor = successor_lineage();
        let handoff = handoff();
        let viability = viability();

        let mut wrong_run = experiment();
        wrong_run.symtropy_run_binding = "symtropy-pr:766:other".into();
        assert!(verify_regenerative_lineage_experiment_evidence(
            &source,
            &successor,
            &handoff,
            &viability,
            &wrong_run,
        )
        .is_err());

        let mut wrong_tick = experiment();
        wrong_tick.roles[1].first_unavailable_tick = Some(7);
        assert!(verify_regenerative_lineage_experiment_evidence(
            &source,
            &successor,
            &handoff,
            &viability,
            &wrong_tick,
        )
        .is_err());
    }

    #[test]
    fn recovery_must_follow_failure_and_fit_observation_window() {
        let mut invalid = experiment();
        invalid.roles[2].first_recovery_tick = Some(5);
        assert!(invalid.validate().is_err());

        let mut out_of_window = experiment();
        out_of_window.roles[2].first_recovery_tick = Some(10);
        assert!(out_of_window.validate().is_err());
    }

    #[test]
    fn lost_lineage_outcome_requires_reproduction_loss_before_operation_failure() {
        let mut invalid = experiment();
        invalid.roles[1].first_unavailable_tick = Some(9);
        invalid.roles[2].first_unavailable_tick = Some(9);
        invalid.roles[2].first_recovery_tick = None;
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn experiment_evidence_fits_existing_maritime_transport() {
        let envelope = experiment()
            .to_maritime_envelope(
                "manta-civil-demo-01",
                6,
                0,
                1_789_100_000_000_000,
                "evidence:manta-forge-lineage-experiment-v1",
            )
            .unwrap();
        assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
        let decoded: RegenerativeLineageExperimentEvidenceV1 =
            serde_json::from_str(&envelope.payload_json).unwrap();
        assert_eq!(decoded, experiment());
    }
}
