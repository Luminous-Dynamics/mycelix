// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Field qualification evidence contracts for read-only Mycelix business pilots.
//!
//! Passing this gate is evidence about one profile/capability/scope under one preregistered
//! protocol. It grants no authority and does not establish causal financial impact.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{CapabilityRef, Digest32, ProfileRef, ReferenceId, ScopeRef};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConnectorBinding {
    pub source_system: ReferenceId,
    pub adapter_semantic_id: ReferenceId,
    pub adapter_digest: Digest32,
    pub mapping_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DataQualityThreshold {
    pub input: ReferenceId,
    pub maximum_missing_bps: u16,
    pub maximum_conflicting_bps: u16,
    pub maximum_stale_bps: u16,
    pub maximum_ingest_delay_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EvaluationSlice {
    pub slice: ReferenceId,
    /// Digest of the externally defined deterministic slice predicate.
    pub criterion_digest: Digest32,
    pub minimum_cases: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FieldQualificationPlan {
    pub plan_id: ReferenceId,
    pub profile: ProfileRef,
    pub capability: CapabilityRef,
    pub scope: ScopeRef,
    pub shadow_protocol_digest: Digest32,
    pub connectors: Vec<ConnectorBinding>,
    pub data_quality: Vec<DataQualityThreshold>,
    pub slices: Vec<EvaluationSlice>,
    pub registered_at_unix_ms: u64,
    pub evaluation_start_unix_ms: u64,
    pub evaluation_end_unix_ms: u64,
    pub plan_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlanError {
    ZeroPlanDigest,
    ZeroShadowProtocolDigest,
    InvalidEvaluationWindow,
    NotPreregistered,
    NoConnectors,
    NoDataQualityThresholds,
    NoSlices,
    ZeroConnectorDigest { source_system: ReferenceId },
    ZeroMappingDigest { source_system: ReferenceId },
    DuplicateConnector { source_system: ReferenceId },
    InvalidBasisPoints { input: ReferenceId },
    ZeroMaximumIngestDelay { input: ReferenceId },
    DuplicateDataQualityInput { input: ReferenceId },
    ZeroSliceDigest { slice: ReferenceId },
    ZeroSliceMinimumCases { slice: ReferenceId },
    DuplicateSlice { slice: ReferenceId },
}

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

impl FieldQualificationPlan {
    pub fn validate(&self) -> Result<(), PlanError> {
        if zero_digest(&self.plan_digest) {
            return Err(PlanError::ZeroPlanDigest);
        }
        if zero_digest(&self.shadow_protocol_digest) {
            return Err(PlanError::ZeroShadowProtocolDigest);
        }
        if self.evaluation_start_unix_ms == 0
            || self.evaluation_start_unix_ms >= self.evaluation_end_unix_ms
        {
            return Err(PlanError::InvalidEvaluationWindow);
        }
        if self.registered_at_unix_ms == 0
            || self.registered_at_unix_ms >= self.evaluation_start_unix_ms
        {
            return Err(PlanError::NotPreregistered);
        }
        if self.connectors.is_empty() {
            return Err(PlanError::NoConnectors);
        }
        if self.data_quality.is_empty() {
            return Err(PlanError::NoDataQualityThresholds);
        }
        if self.slices.is_empty() {
            return Err(PlanError::NoSlices);
        }

        let mut connector_sources = BTreeSet::new();
        for connector in &self.connectors {
            if zero_digest(&connector.adapter_digest) {
                return Err(PlanError::ZeroConnectorDigest {
                    source_system: connector.source_system.clone(),
                });
            }
            if zero_digest(&connector.mapping_digest) {
                return Err(PlanError::ZeroMappingDigest {
                    source_system: connector.source_system.clone(),
                });
            }
            if !connector_sources.insert(connector.source_system.clone()) {
                return Err(PlanError::DuplicateConnector {
                    source_system: connector.source_system.clone(),
                });
            }
        }

        let mut quality_inputs = BTreeSet::new();
        for threshold in &self.data_quality {
            if threshold.maximum_missing_bps > 10_000
                || threshold.maximum_conflicting_bps > 10_000
                || threshold.maximum_stale_bps > 10_000
            {
                return Err(PlanError::InvalidBasisPoints {
                    input: threshold.input.clone(),
                });
            }
            if threshold.maximum_ingest_delay_ms == 0 {
                return Err(PlanError::ZeroMaximumIngestDelay {
                    input: threshold.input.clone(),
                });
            }
            if !quality_inputs.insert(threshold.input.clone()) {
                return Err(PlanError::DuplicateDataQualityInput {
                    input: threshold.input.clone(),
                });
            }
        }

        let mut slices = BTreeSet::new();
        for slice in &self.slices {
            if zero_digest(&slice.criterion_digest) {
                return Err(PlanError::ZeroSliceDigest {
                    slice: slice.slice.clone(),
                });
            }
            if slice.minimum_cases == 0 {
                return Err(PlanError::ZeroSliceMinimumCases {
                    slice: slice.slice.clone(),
                });
            }
            if !slices.insert(slice.slice.clone()) {
                return Err(PlanError::DuplicateSlice {
                    slice: slice.slice.clone(),
                });
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DataQualityEvidence {
    pub input: ReferenceId,
    pub expected_records: u64,
    pub missing_records: u64,
    pub conflicting_records: u64,
    pub stale_records: u64,
    pub maximum_observed_ingest_delay_ms: u64,
    pub evidence_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SliceEvidence {
    pub slice: ReferenceId,
    pub criterion_digest: Digest32,
    pub cases: u32,
    pub shadow_gate_passed: bool,
    pub evidence_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FieldQualificationEvidence {
    pub plan_digest: Digest32,
    pub profile: ProfileRef,
    pub capability: CapabilityRef,
    pub scope: ScopeRef,
    pub shadow_protocol_digest: Digest32,
    /// Exact connector/mapping identities observed during evaluation.
    pub connectors: Vec<ConnectorBinding>,
    pub data_quality: Vec<DataQualityEvidence>,
    pub slices: Vec<SliceEvidence>,
    pub known_limitations: BTreeSet<ReferenceId>,
    pub evidence_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EvidenceError {
    InvalidPlan(PlanError),
    PlanDigestMismatch,
    ProfileMismatch,
    CapabilityMismatch,
    ScopeMismatch,
    ShadowProtocolMismatch,
    ZeroEvidenceDigest,
    ConnectorSetMismatch,
    DuplicateQualityEvidence { input: ReferenceId },
    MissingQualityEvidence { input: ReferenceId },
    ZeroExpectedRecords { input: ReferenceId },
    QualityCountExceedsExpected { input: ReferenceId },
    ZeroQualityEvidenceDigest { input: ReferenceId },
    DuplicateSliceEvidence { slice: ReferenceId },
    MissingSliceEvidence { slice: ReferenceId },
    SliceCriterionMismatch { slice: ReferenceId },
    ZeroSliceEvidenceDigest { slice: ReferenceId },
}

impl FieldQualificationEvidence {
    fn validate_against(&self, plan: &FieldQualificationPlan) -> Result<(), EvidenceError> {
        plan.validate().map_err(EvidenceError::InvalidPlan)?;
        if self.plan_digest != plan.plan_digest {
            return Err(EvidenceError::PlanDigestMismatch);
        }
        if self.profile != plan.profile {
            return Err(EvidenceError::ProfileMismatch);
        }
        if self.capability != plan.capability {
            return Err(EvidenceError::CapabilityMismatch);
        }
        if self.scope != plan.scope {
            return Err(EvidenceError::ScopeMismatch);
        }
        if self.shadow_protocol_digest != plan.shadow_protocol_digest {
            return Err(EvidenceError::ShadowProtocolMismatch);
        }
        if zero_digest(&self.evidence_digest) {
            return Err(EvidenceError::ZeroEvidenceDigest);
        }

        let planned_connectors = plan
            .connectors
            .iter()
            .map(|value| (value.source_system.clone(), value.clone()))
            .collect::<BTreeMap<_, _>>();
        let actual_connectors = self
            .connectors
            .iter()
            .map(|value| (value.source_system.clone(), value.clone()))
            .collect::<BTreeMap<_, _>>();
        if planned_connectors.len() != plan.connectors.len()
            || actual_connectors.len() != self.connectors.len()
            || planned_connectors != actual_connectors
        {
            return Err(EvidenceError::ConnectorSetMismatch);
        }

        let mut quality = BTreeMap::new();
        for item in &self.data_quality {
            if quality.insert(item.input.clone(), item).is_some() {
                return Err(EvidenceError::DuplicateQualityEvidence {
                    input: item.input.clone(),
                });
            }
            if item.expected_records == 0 {
                return Err(EvidenceError::ZeroExpectedRecords {
                    input: item.input.clone(),
                });
            }
            if item.missing_records > item.expected_records
                || item.conflicting_records > item.expected_records
                || item.stale_records > item.expected_records
            {
                return Err(EvidenceError::QualityCountExceedsExpected {
                    input: item.input.clone(),
                });
            }
            if zero_digest(&item.evidence_digest) {
                return Err(EvidenceError::ZeroQualityEvidenceDigest {
                    input: item.input.clone(),
                });
            }
        }
        for threshold in &plan.data_quality {
            if !quality.contains_key(&threshold.input) {
                return Err(EvidenceError::MissingQualityEvidence {
                    input: threshold.input.clone(),
                });
            }
        }

        let mut slices = BTreeMap::new();
        for item in &self.slices {
            if slices.insert(item.slice.clone(), item).is_some() {
                return Err(EvidenceError::DuplicateSliceEvidence {
                    slice: item.slice.clone(),
                });
            }
            if zero_digest(&item.evidence_digest) {
                return Err(EvidenceError::ZeroSliceEvidenceDigest {
                    slice: item.slice.clone(),
                });
            }
        }
        for required in &plan.slices {
            let Some(actual) = slices.get(&required.slice) else {
                return Err(EvidenceError::MissingSliceEvidence {
                    slice: required.slice.clone(),
                });
            };
            if actual.criterion_digest != required.criterion_digest {
                return Err(EvidenceError::SliceCriterionMismatch {
                    slice: required.slice.clone(),
                });
            }
        }
        Ok(())
    }
}

fn basis_points(count: u64, total: u64) -> u16 {
    if total == 0 {
        return 10_000;
    }
    ((u128::from(count) * 10_000) / u128::from(total)).min(10_000) as u16
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FieldQualificationDecision {
    PassShadowFieldGate,
    DataQualityFailed {
        input: ReferenceId,
        dimension: ReferenceId,
        actual: u64,
        maximum: u64,
    },
    InsufficientSliceCases {
        slice: ReferenceId,
        actual: u32,
        minimum: u32,
    },
    SliceShadowGateFailed {
        slice: ReferenceId,
    },
}

pub fn evaluate_field_qualification(
    plan: &FieldQualificationPlan,
    evidence: &FieldQualificationEvidence,
) -> Result<FieldQualificationDecision, EvidenceError> {
    evidence.validate_against(plan)?;

    let quality = evidence
        .data_quality
        .iter()
        .map(|value| (value.input.clone(), value))
        .collect::<BTreeMap<_, _>>();
    for threshold in &plan.data_quality {
        let item = quality[&threshold.input];
        let missing_bps = basis_points(item.missing_records, item.expected_records);
        if missing_bps > threshold.maximum_missing_bps {
            return Ok(FieldQualificationDecision::DataQualityFailed {
                input: threshold.input.clone(),
                dimension: ReferenceId::new("quality:missing-bps").expect("static id"),
                actual: u64::from(missing_bps),
                maximum: u64::from(threshold.maximum_missing_bps),
            });
        }
        let conflicting_bps = basis_points(item.conflicting_records, item.expected_records);
        if conflicting_bps > threshold.maximum_conflicting_bps {
            return Ok(FieldQualificationDecision::DataQualityFailed {
                input: threshold.input.clone(),
                dimension: ReferenceId::new("quality:conflicting-bps").expect("static id"),
                actual: u64::from(conflicting_bps),
                maximum: u64::from(threshold.maximum_conflicting_bps),
            });
        }
        let stale_bps = basis_points(item.stale_records, item.expected_records);
        if stale_bps > threshold.maximum_stale_bps {
            return Ok(FieldQualificationDecision::DataQualityFailed {
                input: threshold.input.clone(),
                dimension: ReferenceId::new("quality:stale-bps").expect("static id"),
                actual: u64::from(stale_bps),
                maximum: u64::from(threshold.maximum_stale_bps),
            });
        }
        if item.maximum_observed_ingest_delay_ms > threshold.maximum_ingest_delay_ms {
            return Ok(FieldQualificationDecision::DataQualityFailed {
                input: threshold.input.clone(),
                dimension: ReferenceId::new("quality:ingest-delay-ms").expect("static id"),
                actual: item.maximum_observed_ingest_delay_ms,
                maximum: threshold.maximum_ingest_delay_ms,
            });
        }
    }

    let slices = evidence
        .slices
        .iter()
        .map(|value| (value.slice.clone(), value))
        .collect::<BTreeMap<_, _>>();
    for required in &plan.slices {
        let actual = slices[&required.slice];
        if actual.cases < required.minimum_cases {
            return Ok(FieldQualificationDecision::InsufficientSliceCases {
                slice: required.slice.clone(),
                actual: actual.cases,
                minimum: required.minimum_cases,
            });
        }
        if !actual.shadow_gate_passed {
            return Ok(FieldQualificationDecision::SliceShadowGateFailed {
                slice: required.slice.clone(),
            });
        }
    }

    Ok(FieldQualificationDecision::PassShadowFieldGate)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn plan() -> FieldQualificationPlan {
        FieldQualificationPlan {
            plan_id: id("field-plan:1"),
            profile: ProfileRef(id("profile:hospitality:food-service:v1")),
            capability: CapabilityRef(id("forecast:hospitality:demand:v1")),
            scope: ScopeRef(id("scope:location:a")),
            shadow_protocol_digest: Digest32::repeat(1),
            connectors: vec![ConnectorBinding {
                source_system: id("source:pos"),
                adapter_semantic_id: id("adapter:pos:v1"),
                adapter_digest: Digest32::repeat(2),
                mapping_digest: Digest32::repeat(3),
            }],
            data_quality: vec![DataQualityThreshold {
                input: id("input:sales"),
                maximum_missing_bps: 100,
                maximum_conflicting_bps: 50,
                maximum_stale_bps: 100,
                maximum_ingest_delay_ms: 60_000,
            }],
            slices: vec![EvaluationSlice {
                slice: id("slice:breakfast"),
                criterion_digest: Digest32::repeat(4),
                minimum_cases: 20,
            }],
            registered_at_unix_ms: 1_000,
            evaluation_start_unix_ms: 2_000,
            evaluation_end_unix_ms: 4_000,
            plan_digest: Digest32::repeat(5),
        }
    }

    fn evidence() -> FieldQualificationEvidence {
        let plan = plan();
        FieldQualificationEvidence {
            plan_digest: plan.plan_digest,
            profile: plan.profile,
            capability: plan.capability,
            scope: plan.scope,
            shadow_protocol_digest: plan.shadow_protocol_digest,
            connectors: plan.connectors,
            data_quality: vec![DataQualityEvidence {
                input: id("input:sales"),
                expected_records: 1_000,
                missing_records: 5,
                conflicting_records: 1,
                stale_records: 3,
                maximum_observed_ingest_delay_ms: 30_000,
                evidence_digest: Digest32::repeat(6),
            }],
            slices: vec![SliceEvidence {
                slice: id("slice:breakfast"),
                criterion_digest: Digest32::repeat(4),
                cases: 30,
                shadow_gate_passed: true,
                evidence_digest: Digest32::repeat(7),
            }],
            known_limitations: BTreeSet::from([id("limitation:single-location")]),
            evidence_digest: Digest32::repeat(8),
        }
    }

    #[test]
    fn complete_evidence_passes_shadow_field_gate() {
        assert_eq!(
            evaluate_field_qualification(&plan(), &evidence()),
            Ok(FieldQualificationDecision::PassShadowFieldGate)
        );
    }

    #[test]
    fn missingness_cannot_be_silently_excluded() {
        let mut evidence = evidence();
        evidence.data_quality[0].missing_records = 11;
        assert_eq!(
            evaluate_field_qualification(&plan(), &evidence),
            Ok(FieldQualificationDecision::DataQualityFailed {
                input: id("input:sales"),
                dimension: id("quality:missing-bps"),
                actual: 110,
                maximum: 100,
            })
        );
    }

    #[test]
    fn aggregate_pass_cannot_hide_underpowered_required_slice() {
        let mut evidence = evidence();
        evidence.slices[0].cases = 19;
        assert_eq!(
            evaluate_field_qualification(&plan(), &evidence),
            Ok(FieldQualificationDecision::InsufficientSliceCases {
                slice: id("slice:breakfast"),
                actual: 19,
                minimum: 20,
            })
        );
    }

    #[test]
    fn connector_mapping_drift_invalidates_evidence() {
        let mut evidence = evidence();
        evidence.connectors[0].mapping_digest = Digest32::repeat(9);
        assert_eq!(
            evaluate_field_qualification(&plan(), &evidence),
            Err(EvidenceError::ConnectorSetMismatch)
        );
    }

    #[test]
    fn slice_definition_cannot_change_after_registration() {
        let mut evidence = evidence();
        evidence.slices[0].criterion_digest = Digest32::repeat(10);
        assert_eq!(
            evaluate_field_qualification(&plan(), &evidence),
            Err(EvidenceError::SliceCriterionMismatch {
                slice: id("slice:breakfast"),
            })
        );
    }
}
