// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Public AC-009 semantic boundary for identity-resolution sensitivity.
//!
//! The internal engine validates exact arithmetic and lineage. This facade first
//! binds that engine to the exact metric family it was designed to explain, so a
//! caller cannot relabel supplier-HHI bytes as another capture metric while
//! retaining the same method reference.

use serde::{Deserialize, Serialize};

use crate::capture_observation::CaptureMetric;
use crate::equivalence_view::IdentityProjectedObservation;
use crate::identity_sensitivity::{
    IdentityProjectionSensitivityReport,
    IdentitySensitivityContract as InternalIdentitySensitivityContract,
    IdentitySensitivityError,
};
use crate::institutional_graph::InstitutionalEdge;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum QualifiedIdentitySensitivityError {
    BaselineMetricKindMismatch,
    ProjectedMetricKindMismatch,
    Analysis(IdentitySensitivityError),
}

#[derive(Debug, Default, Clone, Copy)]
pub struct IdentitySensitivityContract;

impl IdentitySensitivityContract {
    pub fn analyze_procurement_supplier_projection(
        edges: &[InstitutionalEdge],
        projection: &IdentityProjectedObservation,
    ) -> Result<IdentityProjectionSensitivityReport, Vec<QualifiedIdentitySensitivityError>> {
        let mut errors = Vec::new();
        if projection.baseline.observation.measurement.metric
            != CaptureMetric::ProcurementSupplierConcentration
        {
            errors.push(QualifiedIdentitySensitivityError::BaselineMetricKindMismatch);
        }
        if projection.projected.observation.measurement.metric
            != CaptureMetric::ProcurementSupplierConcentration
        {
            errors.push(QualifiedIdentitySensitivityError::ProjectedMetricKindMismatch);
        }
        if !errors.is_empty() {
            return Err(errors);
        }

        InternalIdentitySensitivityContract::analyze_procurement_supplier_projection(
            edges,
            projection,
        )
        .map_err(|internal| {
            internal
                .into_iter()
                .map(QualifiedIdentitySensitivityError::Analysis)
                .collect()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture_metrics::{DerivedCaptureObservation, ObservationContext};
    use crate::capture_observation::{
        CaptureObservation, CaptureSubject, ConfidenceAssessment, ConfidenceLevel, Measurement,
        MetricValue, Uncertainty,
    };
    use crate::equivalence_view::IdentityProjectedObservation;

    fn context() -> ObservationContext {
        ObservationContext {
            observation_id: "projection:test".into(),
            subject: CaptureSubject::ContractingProcedure("test".into()),
            observed_at: 1,
            confidence: ConfidenceAssessment::Qualitative {
                level: ConfidenceLevel::Moderate,
                basis: "test".into(),
            },
            limitations: vec!["test".into()],
        }
    }

    fn derived(metric: CaptureMetric) -> DerivedCaptureObservation {
        let ctx = context();
        DerivedCaptureObservation {
            observation: CaptureObservation {
                id: ctx.observation_id,
                subject: ctx.subject,
                measurement: Measurement {
                    metric,
                    value: MetricValue {
                        numerator: 1,
                        denominator: 1,
                        unit: "hhi_ratio".into(),
                    },
                    method_ref: "mycelix:ac-004:procurement-supplier-hhi:v1".into(),
                },
                provenance: vec![],
                uncertainty: Uncertainty {
                    confidence: ctx.confidence,
                    limitations: ctx.limitations,
                },
                observed_at: ctx.observed_at,
            },
            input_edge_refs: vec![],
        }
    }

    #[test]
    fn relabeled_baseline_metric_is_rejected_before_internal_analysis() {
        let projection = IdentityProjectedObservation {
            projection_id: "projection:test".into(),
            baseline: derived(CaptureMetric::AuthorityConcentration),
            projected: derived(CaptureMetric::ProcurementSupplierConcentration),
            applied_components: vec![],
            applied_identity_link_refs: vec![],
            projection_method_ref: "mycelix:ac-007:qualified-entity-equivalence-view:v1".into(),
        };
        assert_eq!(
            IdentitySensitivityContract::analyze_procurement_supplier_projection(&[], &projection),
            Err(vec![QualifiedIdentitySensitivityError::BaselineMetricKindMismatch])
        );
    }

    #[test]
    fn relabeled_projected_metric_is_rejected_before_internal_analysis() {
        let projection = IdentityProjectedObservation {
            projection_id: "projection:test".into(),
            baseline: derived(CaptureMetric::ProcurementSupplierConcentration),
            projected: derived(CaptureMetric::EvidenceDeficit),
            applied_components: vec![],
            applied_identity_link_refs: vec![],
            projection_method_ref: "mycelix:ac-007:qualified-entity-equivalence-view:v1".into(),
        };
        assert_eq!(
            IdentitySensitivityContract::analyze_procurement_supplier_projection(&[], &projection),
            Err(vec![QualifiedIdentitySensitivityError::ProjectedMetricKindMismatch])
        );
    }
}
