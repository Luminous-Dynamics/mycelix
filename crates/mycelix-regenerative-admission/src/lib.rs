// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Domain-neutral admission of existing PEF evidence into regenerative domains.
//!
//! This crate owns one narrow theorem: raw PEF evidence may be admitted only for
//! `Reported` / `Observed`, while computed PEF evidence must arrive through a
//! validated `LineagedObservation`. Domain-specific semantics such as soil role,
//! biomass sustainability, contamination safety, currentness, rights, or action
//! authority remain outside this crate.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_core_types::{
    EnvironmentalObservation, EvidenceClass, LineagedObservation, MAX_ID_BYTES,
};
use std::fmt;

/// Exact domain expectation applied after the owning PEF validator succeeds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EvidenceExpectation<'a> {
    observation_id: &'a str,
    phenomenon: &'a str,
    evidence_class: Option<EvidenceClass>,
}

impl<'a> EvidenceExpectation<'a> {
    /// Construct and validate an exact PEF expectation.
    pub fn new(
        observation_id: &'a str,
        phenomenon: &'a str,
        evidence_class: Option<EvidenceClass>,
    ) -> Result<Self, EvidenceAdmissionError> {
        require_expectation_text("expectation.observation_id", observation_id)?;
        require_expectation_text("expectation.phenomenon", phenomenon)?;
        Ok(Self {
            observation_id,
            phenomenon,
            evidence_class,
        })
    }

    /// Exact opaque PEF observation identifier expected by the caller.
    pub fn observation_id(&self) -> &str {
        self.observation_id
    }

    /// Exact PEF phenomenon expected by the caller.
    pub fn phenomenon(&self) -> &str {
        self.phenomenon
    }

    /// Optional exact PEF evidence class expected by the caller.
    pub fn evidence_class(&self) -> Option<EvidenceClass> {
        self.evidence_class
    }
}

/// Candidate PEF evidence supplied by a domain-specific resolver.
#[derive(Debug, Clone, Copy)]
pub enum EvidenceCandidate<'a> {
    /// Raw `Reported` / `Observed` evidence.
    Raw(&'a EnvironmentalObservation),
    /// Computed evidence carrying its PEF provenance DAG.
    Lineaged(&'a LineagedObservation),
}

/// Validate and admit one PEF evidence candidate against an exact expectation.
///
/// This function performs no I/O and makes no domain-specific suitability,
/// freshness, representativeness, rights, recommendation, or authority decision.
pub fn admit_pef_evidence<'a>(
    expectation: &EvidenceExpectation<'_>,
    candidate: EvidenceCandidate<'a>,
) -> Result<&'a EnvironmentalObservation, EvidenceAdmissionError> {
    let observation = match candidate {
        EvidenceCandidate::Raw(observation) => {
            observation
                .validate()
                .map_err(|error| EvidenceAdmissionError::InvalidEvidence {
                    observation_id: expectation.observation_id.to_owned(),
                    reason: error.to_string(),
                })?;
            if !matches!(
                observation.class,
                EvidenceClass::Reported | EvidenceClass::Observed
            ) {
                return Err(EvidenceAdmissionError::ComputedEvidenceRequiresLineage {
                    observation_id: expectation.observation_id.to_owned(),
                    actual: observation.class,
                });
            }
            observation
        }
        EvidenceCandidate::Lineaged(product) => {
            product
                .validate()
                .map_err(|error| EvidenceAdmissionError::InvalidEvidence {
                    observation_id: expectation.observation_id.to_owned(),
                    reason: error.to_string(),
                })?;
            &product.observation
        }
    };

    if observation.id != expectation.observation_id {
        return Err(EvidenceAdmissionError::ObservationIdMismatch {
            expected: expectation.observation_id.to_owned(),
            actual: observation.id.clone(),
        });
    }
    if observation.phenomenon != expectation.phenomenon {
        return Err(EvidenceAdmissionError::PhenomenonMismatch {
            observation_id: expectation.observation_id.to_owned(),
            expected: expectation.phenomenon.to_owned(),
            actual: observation.phenomenon.clone(),
        });
    }
    if let Some(expected) = expectation.evidence_class
        && observation.class != expected
    {
        return Err(EvidenceAdmissionError::EvidenceClassMismatch {
            observation_id: expectation.observation_id.to_owned(),
            expected,
            actual: observation.class,
        });
    }

    Ok(observation)
}

/// Shared evidence-admission failures.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EvidenceAdmissionError {
    /// An expectation field is empty or whitespace-only.
    EmptyExpectation(&'static str),
    /// An expectation field exceeds the PEF identifier byte bound.
    ExpectationTooLong {
        /// Field name.
        field: &'static str,
        /// Actual UTF-8 byte length.
        actual: usize,
        /// Maximum UTF-8 byte length.
        max: usize,
    },
    /// Candidate evidence failed its owning PEF validator.
    InvalidEvidence {
        /// Exact observation ID requested by the caller.
        observation_id: String,
        /// Validation failure rendered by the owning PEF type.
        reason: String,
    },
    /// A computed class was supplied as a raw observation without lineage.
    ComputedEvidenceRequiresLineage {
        /// Exact observation ID requested by the caller.
        observation_id: String,
        /// Computed class requiring lineage.
        actual: EvidenceClass,
    },
    /// The candidate resolved to a different exact PEF observation ID.
    ObservationIdMismatch {
        /// Exact expected ID.
        expected: String,
        /// Exact candidate ID.
        actual: String,
    },
    /// The right ID carried a different phenomenon.
    PhenomenonMismatch {
        /// Exact PEF observation ID.
        observation_id: String,
        /// Exact expected phenomenon.
        expected: String,
        /// Actual candidate phenomenon.
        actual: String,
    },
    /// The candidate evidence class did not match an explicit expectation.
    EvidenceClassMismatch {
        /// Exact PEF observation ID.
        observation_id: String,
        /// Expected evidence class.
        expected: EvidenceClass,
        /// Actual evidence class.
        actual: EvidenceClass,
    },
}

impl fmt::Display for EvidenceAdmissionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyExpectation(field) => write!(f, "{field} cannot be empty"),
            Self::ExpectationTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::InvalidEvidence {
                observation_id,
                reason,
            } => write!(f, "invalid PEF evidence {observation_id}: {reason}"),
            Self::ComputedEvidenceRequiresLineage {
                observation_id,
                actual,
            } => write!(
                f,
                "computed PEF evidence {observation_id} with class {actual:?} requires LineagedObservation"
            ),
            Self::ObservationIdMismatch { expected, actual } => write!(
                f,
                "PEF observation id {actual} does not exactly match expected {expected}"
            ),
            Self::PhenomenonMismatch {
                observation_id,
                expected,
                actual,
            } => write!(
                f,
                "PEF observation {observation_id} phenomenon {actual} does not match expected {expected}"
            ),
            Self::EvidenceClassMismatch {
                observation_id,
                expected,
                actual,
            } => write!(
                f,
                "PEF observation {observation_id} class {actual:?} does not match expected {expected:?}"
            ),
        }
    }
}

impl std::error::Error for EvidenceAdmissionError {}

fn require_expectation_text(
    field: &'static str,
    value: &str,
) -> Result<(), EvidenceAdmissionError> {
    if value.trim().is_empty() {
        return Err(EvidenceAdmissionError::EmptyExpectation(field));
    }
    if value.len() > MAX_ID_BYTES {
        return Err(EvidenceAdmissionError::ExpectationTooLong {
            field,
            actual: value.len(),
            max: MAX_ID_BYTES,
        });
    }
    Ok(())
}
