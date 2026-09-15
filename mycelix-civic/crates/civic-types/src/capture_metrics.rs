// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-004 deterministic institutional capture metrics.
//!
//! AC-004 converts validated AC-003 graph relationships into AC-002 observations.
//! It does not create findings, sanctions, or guilt labels. Every output preserves
//! the exact graph-edge population used for the calculation.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::capture_observation::{
    CaptureContract, CaptureContractViolation, CaptureMetric, CaptureObservation, CaptureSubject,
    ConfidenceAssessment, Measurement, MetricValue, ProvenanceRef, Uncertainty,
};
use crate::institutional_graph::{
    AssertionStatus, InstitutionalEdge, InstitutionalGraphContract, InstitutionalGraphViolation,
    InstitutionalRelationKind, ProcurementRole,
};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ObservationContext {
    pub observation_id: String,
    pub subject: CaptureSubject,
    pub observed_at: u64,
    pub confidence: ConfidenceAssessment,
    pub limitations: Vec<String>,
}

/// AC-004 output with exact derivation lineage.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct DerivedCaptureObservation {
    pub observation: CaptureObservation,
    pub input_edge_refs: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MetricComputationError {
    EmptyPopulation,
    InvalidInputEdge {
        index: usize,
        violations: Vec<InstitutionalGraphViolation>,
    },
    ArithmeticOverflow,
    MissingInputEdgeReference,
    OutputContractViolation(Vec<CaptureContractViolation>),
}

#[derive(Debug, Default, Clone, Copy)]
pub struct CaptureMetricEngine;

impl CaptureMetricEngine {
    /// Herfindahl-Hirschman concentration over award relationships, weighting each
    /// award edge equally. The exact ratio is sum(count_i^2) / total_awards^2.
    pub fn procurement_supplier_concentration(
        edges: &[InstitutionalEdge],
        context: ObservationContext,
    ) -> Result<DerivedCaptureObservation, MetricComputationError> {
        let population: Vec<_> = edges
            .iter()
            .enumerate()
            .filter(|(_, edge)| {
                matches!(
                    &edge.relation,
                    InstitutionalRelationKind::ProcurementParticipation {
                        role: ProcurementRole::Awardee
                    }
                )
            })
            .collect();

        let counts = validate_and_group_by_actor(&population)?;
        let value = concentration_ratio(&counts)?;
        build_derived_observation(
            population,
            context,
            CaptureMetric::ProcurementSupplierConcentration,
            value,
            "mycelix:ac-004:procurement-supplier-hhi:v1",
            "each award relationship is weighted equally; contract value and contract complexity are not weighted",
        )
    }

    /// Herfindahl-Hirschman concentration over authority-grant relationships,
    /// weighting each authority edge equally by grant holder.
    pub fn authority_grant_concentration(
        edges: &[InstitutionalEdge],
        context: ObservationContext,
    ) -> Result<DerivedCaptureObservation, MetricComputationError> {
        let population: Vec<_> = edges
            .iter()
            .enumerate()
            .filter(|(_, edge)| {
                matches!(
                    &edge.relation,
                    InstitutionalRelationKind::Authority { .. }
                )
            })
            .collect();

        let counts = validate_and_group_by_actor(&population)?;
        let value = concentration_ratio(&counts)?;
        build_derived_observation(
            population,
            context,
            CaptureMetric::AuthorityConcentration,
            value,
            "mycelix:ac-004:authority-grant-hhi:v1",
            "each authority relationship is weighted equally; mandate scope, budget, and practical importance are not weighted",
        )
    }

    /// Share of valid graph assertions that are not in AC-003's `Corroborated`
    /// state. This is an evidence-coverage metric, not a truth or corruption score.
    pub fn evidence_deficit_share(
        edges: &[InstitutionalEdge],
        context: ObservationContext,
    ) -> Result<DerivedCaptureObservation, MetricComputationError> {
        if edges.is_empty() {
            return Err(MetricComputationError::EmptyPopulation);
        }

        let population: Vec<_> = edges.iter().enumerate().collect();
        validate_population(&population)?;

        let uncorroborated = population
            .iter()
            .filter(|(_, edge)| edge.assertion_status != AssertionStatus::Corroborated)
            .count();

        let numerator = i64::try_from(uncorroborated)
            .map_err(|_| MetricComputationError::ArithmeticOverflow)?;
        let denominator = u64::try_from(population.len())
            .map_err(|_| MetricComputationError::ArithmeticOverflow)?;

        build_derived_observation(
            population,
            context,
            CaptureMetric::EvidenceDeficit,
            MetricValue {
                numerator,
                denominator,
                unit: "share".into(),
            },
            "mycelix:ac-004:uncorroborated-edge-share:v1",
            "corroboration is operationally defined by AC-003 as at least two distinct provenance sources; authoritative single-source records still count as uncorroborated",
        )
    }
}

fn validate_and_group_by_actor(
    population: &[(usize, &InstitutionalEdge)],
) -> Result<BTreeMap<String, u64>, MetricComputationError> {
    if population.is_empty() {
        return Err(MetricComputationError::EmptyPopulation);
    }
    validate_population(population)?;

    let mut counts = BTreeMap::new();
    for (_, edge) in population {
        let counter = counts.entry(edge.from.id.clone()).or_insert(0_u64);
        *counter = counter
            .checked_add(1)
            .ok_or(MetricComputationError::ArithmeticOverflow)?;
    }
    Ok(counts)
}

fn validate_population(
    population: &[(usize, &InstitutionalEdge)],
) -> Result<(), MetricComputationError> {
    if population.is_empty() {
        return Err(MetricComputationError::EmptyPopulation);
    }

    for (index, edge) in population {
        if edge.id.trim().is_empty() {
            return Err(MetricComputationError::MissingInputEdgeReference);
        }
        if let Err(violations) = InstitutionalGraphContract::validate_edge(edge) {
            return Err(MetricComputationError::InvalidInputEdge {
                index: *index,
                violations,
            });
        }
    }
    Ok(())
}

fn concentration_ratio(
    counts: &BTreeMap<String, u64>,
) -> Result<MetricValue, MetricComputationError> {
    let total = counts.values().try_fold(0_u128, |acc, count| {
        acc.checked_add(u128::from(*count))
            .ok_or(MetricComputationError::ArithmeticOverflow)
    })?;
    if total == 0 {
        return Err(MetricComputationError::EmptyPopulation);
    }

    let numerator = counts.values().try_fold(0_u128, |acc, count| {
        let count = u128::from(*count);
        let square = count
            .checked_mul(count)
            .ok_or(MetricComputationError::ArithmeticOverflow)?;
        acc.checked_add(square)
            .ok_or(MetricComputationError::ArithmeticOverflow)
    })?;
    let denominator = total
        .checked_mul(total)
        .ok_or(MetricComputationError::ArithmeticOverflow)?;

    Ok(MetricValue {
        numerator: i64::try_from(numerator)
            .map_err(|_| MetricComputationError::ArithmeticOverflow)?,
        denominator: u64::try_from(denominator)
            .map_err(|_| MetricComputationError::ArithmeticOverflow)?,
        unit: "hhi_ratio".into(),
    })
}

fn build_derived_observation(
    population: Vec<(usize, &InstitutionalEdge)>,
    mut context: ObservationContext,
    metric: CaptureMetric,
    value: MetricValue,
    method_ref: &str,
    fixed_limitation: &str,
) -> Result<DerivedCaptureObservation, MetricComputationError> {
    validate_population(&population)?;

    context.limitations.push(fixed_limitation.into());

    let mut provenance_keys = BTreeSet::new();
    for (_, edge) in &population {
        for provenance in &edge.provenance {
            provenance_keys.insert((
                provenance.source_ref.clone(),
                provenance.content_hash.clone(),
            ));
        }
    }
    let provenance = provenance_keys
        .into_iter()
        .map(|(source_ref, content_hash)| ProvenanceRef {
            source_ref,
            content_hash,
        })
        .collect();

    let input_edge_refs = population
        .iter()
        .map(|(_, edge)| edge.id.clone())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();

    if input_edge_refs.is_empty()
        || input_edge_refs
            .iter()
            .any(|reference| reference.trim().is_empty())
    {
        return Err(MetricComputationError::MissingInputEdgeReference);
    }

    let observation = CaptureObservation {
        id: context.observation_id,
        subject: context.subject,
        measurement: Measurement {
            metric,
            value,
            method_ref: method_ref.into(),
        },
        provenance,
        uncertainty: Uncertainty {
            confidence: context.confidence,
            limitations: context.limitations,
        },
        observed_at: context.observed_at,
    };

    CaptureContract::validate_observation(&observation)
        .map_err(MetricComputationError::OutputContractViolation)?;

    Ok(DerivedCaptureObservation {
        observation,
        input_edge_refs,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::anti_capture::ConsequenceBasis;
    use crate::capture_observation::ConfidenceLevel;
    use crate::institutional_graph::{
        DisclosureClass, InstitutionalNodeKind, InstitutionalNodeRef,
    };

    fn node(id: &str, kind: InstitutionalNodeKind) -> InstitutionalNodeRef {
        InstitutionalNodeRef {
            id: id.into(),
            kind,
        }
    }

    fn provenance(id: &str) -> ProvenanceRef {
        ProvenanceRef {
            source_ref: format!("source:{id}"),
            content_hash: Some(format!("sha256:{id}")),
        }
    }

    fn award(id: &str, supplier: &str) -> InstitutionalEdge {
        InstitutionalEdge {
            id: id.into(),
            from: node(supplier, InstitutionalNodeKind::Organization),
            to: node(
                &format!("procedure:{id}"),
                InstitutionalNodeKind::ProcurementProcedure,
            ),
            relation: InstitutionalRelationKind::ProcurementParticipation {
                role: ProcurementRole::Awardee,
            },
            disclosure: DisclosureClass::PublicMetadata,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![provenance(id)],
            challenge_refs: vec![],
            recorded_at: 1,
            valid_from: None,
            valid_until: None,
        }
    }

    fn authority(id: &str, holder: &str) -> InstitutionalEdge {
        InstitutionalEdge {
            id: id.into(),
            from: node(holder, InstitutionalNodeKind::PublicPowerHolder),
            to: node(
                &format!("office:{id}"),
                InstitutionalNodeKind::PublicOffice,
            ),
            relation: InstitutionalRelationKind::Authority {
                grant_ref: format!("grant:{id}"),
            },
            disclosure: DisclosureClass::PublicMetadata,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![provenance(id)],
            challenge_refs: vec![],
            recorded_at: 1,
            valid_from: Some(1),
            valid_until: Some(2),
        }
    }

    fn context(id: &str) -> ObservationContext {
        ObservationContext {
            observation_id: id.into(),
            subject: CaptureSubject::Process("municipal-procurement".into()),
            observed_at: 100,
            confidence: ConfidenceAssessment::Qualitative {
                level: ConfidenceLevel::Moderate,
                basis: "complete graph-edge population for the declared period".into(),
            },
            limitations: vec!["registry completeness depends on source publication".into()],
        }
    }

    #[test]
    fn supplier_concentration_is_exact_and_reproducible() {
        let edges = vec![
            award("a1", "supplier:a"),
            award("a2", "supplier:a"),
            award("b1", "supplier:b"),
            award("b2", "supplier:b"),
        ];
        let derived = CaptureMetricEngine::procurement_supplier_concentration(
            &edges,
            context("obs:supplier-hhi"),
        )
        .expect("valid metric");

        assert_eq!(derived.observation.measurement.value.numerator, 8);
        assert_eq!(derived.observation.measurement.value.denominator, 16);
        assert_eq!(derived.input_edge_refs.len(), 4);
        assert_eq!(
            derived.observation.constitutional_basis(),
            ConsequenceBasis::Observation
        );
    }

    #[test]
    fn authority_concentration_counts_grants_not_their_unmodeled_importance() {
        let edges = vec![
            authority("g1", "holder:a"),
            authority("g2", "holder:a"),
            authority("g3", "holder:b"),
        ];
        let derived = CaptureMetricEngine::authority_grant_concentration(
            &edges,
            context("obs:authority-hhi"),
        )
        .expect("valid metric");

        assert_eq!(derived.observation.measurement.value.numerator, 5);
        assert_eq!(derived.observation.measurement.value.denominator, 9);
        assert!(derived
            .observation
            .uncertainty
            .limitations
            .iter()
            .any(|item| item.contains("mandate scope")));
    }

    #[test]
    fn evidence_deficit_is_a_coverage_measure_not_a_truth_score() {
        let mut edges = vec![award("a1", "supplier:a"), award("a2", "supplier:b")];
        let mut corroborated = award("a3", "supplier:c");
        corroborated.assertion_status = AssertionStatus::Corroborated;
        corroborated.provenance.push(ProvenanceRef {
            source_ref: "independent:second-source".into(),
            content_hash: Some("sha256:second".into()),
        });
        edges.push(corroborated);

        let derived = CaptureMetricEngine::evidence_deficit_share(
            &edges,
            context("obs:evidence-deficit"),
        )
        .expect("valid metric");

        assert_eq!(derived.observation.measurement.value.numerator, 2);
        assert_eq!(derived.observation.measurement.value.denominator, 3);
    }

    #[test]
    fn invalid_graph_edge_cannot_enter_a_metric_population() {
        let mut edge = award("a1", "supplier:a");
        edge.provenance.clear();
        let error = CaptureMetricEngine::procurement_supplier_concentration(
            &[edge],
            context("obs:invalid"),
        )
        .expect_err("invalid graph input must fail closed");

        assert!(matches!(
            error,
            MetricComputationError::InvalidInputEdge { index: 0, .. }
        ));
    }

    #[test]
    fn empty_population_is_not_reported_as_zero_concentration() {
        assert_eq!(
            CaptureMetricEngine::procurement_supplier_concentration(
                &[],
                context("obs:empty")
            ),
            Err(MetricComputationError::EmptyPopulation)
        );
    }
}
