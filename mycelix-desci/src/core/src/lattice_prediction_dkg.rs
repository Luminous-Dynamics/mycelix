// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Symthaea lattice-prediction commitments for the Mycelix epistemic graph.
//!
//! A prediction commitment and its later reveal are distinct append-only facts.
//! Scoring is derived after reveal; this module deliberately does not convert a
//! normalized residual into an automatic `supports` or `refutes` verdict.

use crate::spectroscopy_dkg::{
    SpectroscopyDkgEdge, SpectroscopyDkgNode, SpectroscopyNodeKind, SpectroscopyProvenance,
    SpectroscopyRelation,
};
use chrono::DateTime;
use serde::{Deserialize, Serialize};

pub const LATTICE_PREDICTION_PROTOCOL: &str = "mycelix-symthaea-lattice-prediction";
pub const LATTICE_PREDICTION_SCHEMA_VERSION: u16 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LatticePredictionMethod {
    FirstPrinciplesLattice,
    ValidatedSurrogate,
    AnalyticReference,
    ExploratoryHeuristic,
}

impl LatticePredictionMethod {
    pub fn can_be_called_lattice_output(self) -> bool {
        matches!(self, Self::FirstPrinciplesLattice)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LatticePredictionCommitment {
    pub protocol: String,
    pub schema_version: u16,
    pub producer: String,
    pub prediction_id: String,
    pub held_out_id: String,
    pub observable: String,
    pub ensemble_id: String,
    pub units: String,
    pub method: LatticePredictionMethod,
    pub value: f64,
    pub uncertainty: f64,
    pub code_revision: String,
    pub model_revision: String,
    pub configuration_digest: String,
    pub training_data_digest: Option<String>,
    pub evidence_lineage: String,
    /// RFC3339 timestamp supplied by the committing evidence service.
    pub frozen_at: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LatticePredictionReveal {
    pub held_out_id: String,
    pub observed_value: f64,
    pub observed_uncertainty: f64,
    pub source_id: String,
    pub evidence_lineage: String,
    /// RFC3339 timestamp for when the held-out result became available.
    pub revealed_at: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LatticePredictionScore {
    pub absolute_error: f64,
    pub combined_uncertainty: f64,
    pub normalized_residual: f64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LatticePredictionDkgError {
    ProtocolMismatch,
    UnsupportedSchema(u16),
    EmptyField(&'static str),
    NonFiniteValue,
    InvalidUncertainty,
    MissingTrainingDigest,
    InvalidTimestamp,
    RevealBeforeCommitment,
    RevealIdMismatch,
}

fn nonempty(value: &str, field: &'static str) -> Result<(), LatticePredictionDkgError> {
    if value.trim().is_empty() {
        Err(LatticePredictionDkgError::EmptyField(field))
    } else {
        Ok(())
    }
}

fn parse_timestamp(value: &str) -> Result<DateTime<chrono::FixedOffset>, LatticePredictionDkgError> {
    DateTime::parse_from_rfc3339(value).map_err(|_| LatticePredictionDkgError::InvalidTimestamp)
}

impl LatticePredictionCommitment {
    pub fn validate(&self) -> Result<(), LatticePredictionDkgError> {
        if self.protocol != LATTICE_PREDICTION_PROTOCOL {
            return Err(LatticePredictionDkgError::ProtocolMismatch);
        }
        if self.schema_version != LATTICE_PREDICTION_SCHEMA_VERSION {
            return Err(LatticePredictionDkgError::UnsupportedSchema(self.schema_version));
        }
        for (value, field) in [
            (&self.producer, "producer"),
            (&self.prediction_id, "prediction_id"),
            (&self.held_out_id, "held_out_id"),
            (&self.observable, "observable"),
            (&self.ensemble_id, "ensemble_id"),
            (&self.units, "units"),
            (&self.code_revision, "code_revision"),
            (&self.model_revision, "model_revision"),
            (&self.configuration_digest, "configuration_digest"),
            (&self.evidence_lineage, "evidence_lineage"),
            (&self.frozen_at, "frozen_at"),
        ] {
            nonempty(value, field)?;
        }
        if !self.value.is_finite() {
            return Err(LatticePredictionDkgError::NonFiniteValue);
        }
        if !self.uncertainty.is_finite() || self.uncertainty <= 0.0 {
            return Err(LatticePredictionDkgError::InvalidUncertainty);
        }
        if self.method == LatticePredictionMethod::ValidatedSurrogate
            && self
                .training_data_digest
                .as_deref()
                .map(str::trim)
                .filter(|value| !value.is_empty())
                .is_none()
        {
            return Err(LatticePredictionDkgError::MissingTrainingDigest);
        }
        parse_timestamp(&self.frozen_at)?;
        Ok(())
    }

    /// Project the immutable prediction record into the generic spectroscopy DKG.
    pub fn as_spectroscopy_node(&self) -> Result<SpectroscopyDkgNode, LatticePredictionDkgError> {
        self.validate()?;
        Ok(SpectroscopyDkgNode {
            id: format!("lattice-prediction::{}", self.prediction_id),
            kind: SpectroscopyNodeKind::LatticePrediction,
            subject: self.ensemble_id.clone(),
            statement: format!(
                "frozen prediction: {} +/- {} {} via {:?}",
                self.value, self.uncertainty, self.units, self.method
            ),
            observable: Some(self.observable.clone()),
            provenance: Some(SpectroscopyProvenance {
                source: self.producer.clone(),
                persistent_id: self.prediction_id.clone(),
                content_hash: None,
            }),
        })
    }
}

impl LatticePredictionReveal {
    pub fn validate_against(
        &self,
        prediction: &LatticePredictionCommitment,
    ) -> Result<(), LatticePredictionDkgError> {
        prediction.validate()?;
        if self.held_out_id != prediction.held_out_id {
            return Err(LatticePredictionDkgError::RevealIdMismatch);
        }
        nonempty(&self.source_id, "source_id")?;
        nonempty(&self.evidence_lineage, "reveal_evidence_lineage")?;
        nonempty(&self.revealed_at, "revealed_at")?;
        if !self.observed_value.is_finite() {
            return Err(LatticePredictionDkgError::NonFiniteValue);
        }
        if !self.observed_uncertainty.is_finite() || self.observed_uncertainty <= 0.0 {
            return Err(LatticePredictionDkgError::InvalidUncertainty);
        }
        let frozen = parse_timestamp(&prediction.frozen_at)?;
        let revealed = parse_timestamp(&self.revealed_at)?;
        if revealed < frozen {
            return Err(LatticePredictionDkgError::RevealBeforeCommitment);
        }
        Ok(())
    }

    pub fn as_spectroscopy_node(
        &self,
        prediction: &LatticePredictionCommitment,
    ) -> Result<SpectroscopyDkgNode, LatticePredictionDkgError> {
        self.validate_against(prediction)?;
        Ok(SpectroscopyDkgNode {
            id: format!("lattice-reveal::{}", self.held_out_id),
            kind: SpectroscopyNodeKind::Observation,
            subject: prediction.ensemble_id.clone(),
            statement: format!(
                "held-out result: {} +/- {} {}",
                self.observed_value, self.observed_uncertainty, prediction.units
            ),
            observable: Some(prediction.observable.clone()),
            provenance: Some(SpectroscopyProvenance {
                source: self.source_id.clone(),
                persistent_id: self.held_out_id.clone(),
                content_hash: None,
            }),
        })
    }
}

pub fn score_lattice_prediction(
    prediction: &LatticePredictionCommitment,
    reveal: &LatticePredictionReveal,
) -> Result<LatticePredictionScore, LatticePredictionDkgError> {
    reveal.validate_against(prediction)?;
    let absolute_error = (prediction.value - reveal.observed_value).abs();
    let combined_uncertainty =
        (prediction.uncertainty.powi(2) + reveal.observed_uncertainty.powi(2)).sqrt();
    Ok(LatticePredictionScore {
        absolute_error,
        combined_uncertainty,
        normalized_residual: absolute_error / combined_uncertainty,
    })
}

/// Preserve prediction/reveal chronology in the generic DKG without inventing
/// an epistemic verdict from an arbitrary residual threshold.
pub fn reveal_context_edge(
    prediction: &LatticePredictionCommitment,
    reveal: &LatticePredictionReveal,
) -> Result<SpectroscopyDkgEdge, LatticePredictionDkgError> {
    reveal.validate_against(prediction)?;
    Ok(SpectroscopyDkgEdge {
        from: format!("lattice-reveal::{}", reveal.held_out_id),
        relation: SpectroscopyRelation::ContextFor,
        to: format!("lattice-prediction::{}", prediction.prediction_id),
        rationale: Some("held-out result used to score the frozen prediction".into()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn prediction() -> LatticePredictionCommitment {
        LatticePredictionCommitment {
            protocol: LATTICE_PREDICTION_PROTOCOL.into(),
            schema_version: LATTICE_PREDICTION_SCHEMA_VERSION,
            producer: "symthaea:lqcd-lineage-1".into(),
            prediction_id: "pred-001".into(),
            held_out_id: "holdout-001".into(),
            observable: "glueball::0++::mass_ratio".into(),
            ensemble_id: "pure-su3-beta-x-volume-y".into(),
            units: "dimensionless".into(),
            method: LatticePredictionMethod::ValidatedSurrogate,
            value: 1.50,
            uncertainty: 0.10,
            code_revision: "abc123".into(),
            model_revision: "surrogate-v1".into(),
            configuration_digest: "sha256:config".into(),
            training_data_digest: Some("sha256:training".into()),
            evidence_lineage: "symthaea-evidence-root-1".into(),
            frozen_at: "2026-09-11T12:00:00Z".into(),
        }
    }

    fn reveal() -> LatticePredictionReveal {
        LatticePredictionReveal {
            held_out_id: "holdout-001".into(),
            observed_value: 1.60,
            observed_uncertainty: 0.10,
            source_id: "independent-reference-run".into(),
            evidence_lineage: "reference-evidence-root-1".into(),
            revealed_at: "2026-09-12T12:00:00Z".into(),
        }
    }

    #[test]
    fn heuristic_never_projects_as_first_principles_lattice_output() {
        assert!(!LatticePredictionMethod::ExploratoryHeuristic.can_be_called_lattice_output());
        assert!(LatticePredictionMethod::FirstPrinciplesLattice.can_be_called_lattice_output());
    }

    #[test]
    fn surrogate_requires_training_digest() {
        let mut p = prediction();
        p.training_data_digest = None;
        assert_eq!(p.validate(), Err(LatticePredictionDkgError::MissingTrainingDigest));
    }

    #[test]
    fn reveal_must_follow_commitment() {
        let p = prediction();
        let mut r = reveal();
        r.revealed_at = "2026-09-10T12:00:00Z".into();
        assert_eq!(
            r.validate_against(&p),
            Err(LatticePredictionDkgError::RevealBeforeCommitment)
        );
    }

    #[test]
    fn scoring_preserves_raw_prediction_and_observation() {
        let score = score_lattice_prediction(&prediction(), &reveal()).unwrap();
        assert!((score.absolute_error - 0.10).abs() < 1e-12);
        assert!((score.combined_uncertainty - 2.0_f64.sqrt() * 0.10).abs() < 1e-12);
        assert!((score.normalized_residual - 1.0 / 2.0_f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn dkg_projection_keeps_prediction_and_reveal_distinct() {
        let p = prediction();
        let r = reveal();
        let prediction_node = p.as_spectroscopy_node().unwrap();
        let reveal_node = r.as_spectroscopy_node(&p).unwrap();
        let edge = reveal_context_edge(&p, &r).unwrap();
        assert_eq!(prediction_node.kind, SpectroscopyNodeKind::LatticePrediction);
        assert_eq!(reveal_node.kind, SpectroscopyNodeKind::Observation);
        assert_eq!(edge.relation, SpectroscopyRelation::ContextFor);
        assert_ne!(prediction_node.id, reveal_node.id);
    }
}
