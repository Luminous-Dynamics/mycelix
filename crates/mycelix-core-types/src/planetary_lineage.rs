// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Validated provenance DAGs for planetary evidence products.
//!
//! `planetary_evidence` answers what an observation is and what source
//! artifacts support it. This module answers a different question:
//! *how was a derived/inferred/forecast/scenario product produced?*
//!
//! The lineage is deliberately authority-free. It records declared roots,
//! transformation/model identity, reproducibility digests, and dependency
//! topology. It does not grant execution authority or claim that a model is
//! scientifically valid merely because its computation is reproducible.

use crate::ExternalEvidenceRef;
use std::{collections::HashSet, fmt};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub const PLANETARY_LINEAGE_SCHEMA_VERSION: u16 = 1;
pub const MAX_LINEAGE_ROOTS: usize = 256;
pub const MAX_LINEAGE_STEPS: usize = 128;
pub const MAX_LINEAGE_INPUTS_PER_STEP: usize = 64;
pub const MAX_LINEAGE_ID_BYTES: usize = 256;
pub const MAX_OPERATION_BYTES: usize = 256;
pub const MAX_IMPLEMENTATION_BYTES: usize = 512;
pub const MAX_VERSION_BYTES: usize = 128;
pub const MAX_LINEAGE_DIGEST_BYTES: usize = 256;

/// A declared root of a provenance graph.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum LineageSource {
    /// Another canonical planetary observation identified by stable ID.
    Observation(String),
    /// A source artifact that has not itself been materialized as a canonical
    /// observation.
    ExternalEvidence(ExternalEvidenceRef),
}

impl LineageSource {
    fn validate(&self, output_observation_id: &str) -> Result<(), PlanetaryLineageError> {
        match self {
            Self::Observation(id) => {
                require_text("lineage.source.observation_id", id, MAX_LINEAGE_ID_BYTES)?;
                if id == output_observation_id {
                    return Err(PlanetaryLineageError::OutputUsedAsInput(id.clone()));
                }
                Ok(())
            }
            Self::ExternalEvidence(reference) => reference
                .validate()
                .map_err(|error| PlanetaryLineageError::InvalidExternalEvidence(error.to_string())),
        }
    }
}

/// Named root so multiple sources can be referenced without copying their
/// complete source identity into every computation step.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct LineageRoot {
    pub id: String,
    pub source: LineageSource,
}

impl LineageRoot {
    pub fn validate(&self, output_observation_id: &str) -> Result<(), PlanetaryLineageError> {
        require_text("lineage.root.id", &self.id, MAX_LINEAGE_ID_BYTES)?;
        self.source.validate(output_observation_id)
    }
}

/// Reference from a computation step to one declared root or an earlier step.
///
/// Step references are intentionally restricted to *earlier* steps in the
/// serialized list. This makes topological order part of the wire contract and
/// makes cycles impossible to encode in a valid lineage.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum LineageRef {
    Root(String),
    Step(String),
}

impl LineageRef {
    fn id(&self) -> &str {
        match self {
            Self::Root(id) | Self::Step(id) => id,
        }
    }
}

/// What kind of producer generated a lineage step.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum ProducerKind {
    DeterministicTransform,
    Aggregation,
    StatisticalModel,
    MachineLearningModel,
    Simulation,
    HumanAssessment,
    ExternalProcess,
}

/// Reproducibility identity for one transformation/model implementation.
///
/// Digests are algorithm-qualified strings such as `sha256:<hex>` or
/// `blake3:<hex>`. The fields are optional because historical/external
/// processes may not expose every capsule component, but absence remains
/// explicit rather than being interpreted as reproducibility.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ProducerIdentity {
    pub kind: ProducerKind,
    pub implementation: String,
    pub version: Option<String>,
    pub code_digest: Option<String>,
    pub configuration_digest: Option<String>,
    pub environment_digest: Option<String>,
}

impl ProducerIdentity {
    pub fn validate(&self) -> Result<(), PlanetaryLineageError> {
        require_text(
            "lineage.producer.implementation",
            &self.implementation,
            MAX_IMPLEMENTATION_BYTES,
        )?;
        if let Some(version) = &self.version {
            require_text("lineage.producer.version", version, MAX_VERSION_BYTES)?;
        }
        for (field, digest) in [
            ("lineage.producer.code_digest", self.code_digest.as_deref()),
            (
                "lineage.producer.configuration_digest",
                self.configuration_digest.as_deref(),
            ),
            (
                "lineage.producer.environment_digest",
                self.environment_digest.as_deref(),
            ),
        ] {
            if let Some(digest) = digest {
                validate_digest(field, digest)?;
            }
        }
        Ok(())
    }

    /// Whether the producer declares the three capsule identities needed to
    /// independently target the same code/configuration/environment.
    ///
    /// This does not prove determinism or scientific validity; it only reports
    /// whether the lineage carries a complete reproducibility identity.
    pub fn has_complete_capsule(&self) -> bool {
        self.code_digest.is_some()
            && self.configuration_digest.is_some()
            && self.environment_digest.is_some()
    }
}

/// One ordered transformation/model step in an evidence lineage.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct LineageStep {
    pub id: String,
    /// Human/computer-readable operation identifier, e.g. `kelvin_to_celsius`,
    /// `daily_mean`, `wrf_v4_forecast`, or `human_expert_review`.
    pub operation: String,
    pub producer: ProducerIdentity,
    pub inputs: Vec<LineageRef>,
    /// Optional digest of this intermediate step's serialized output.
    pub output_digest: Option<String>,
    /// Unix seconds when this step completed, when known.
    pub completed_at: Option<i64>,
}

impl LineageStep {
    fn validate_shape(&self) -> Result<(), PlanetaryLineageError> {
        require_text("lineage.step.id", &self.id, MAX_LINEAGE_ID_BYTES)?;
        require_text(
            "lineage.step.operation",
            &self.operation,
            MAX_OPERATION_BYTES,
        )?;
        self.producer.validate()?;
        if self.inputs.is_empty() {
            return Err(PlanetaryLineageError::MissingInputs(self.id.clone()));
        }
        if self.inputs.len() > MAX_LINEAGE_INPUTS_PER_STEP {
            return Err(PlanetaryLineageError::TooManyInputs {
                step_id: self.id.clone(),
                actual: self.inputs.len(),
                max: MAX_LINEAGE_INPUTS_PER_STEP,
            });
        }
        let mut seen_inputs = HashSet::with_capacity(self.inputs.len());
        for input in &self.inputs {
            require_text("lineage.step.input_id", input.id(), MAX_LINEAGE_ID_BYTES)?;
            if !seen_inputs.insert(input) {
                return Err(PlanetaryLineageError::DuplicateInputRef {
                    step_id: self.id.clone(),
                    input_id: input.id().to_string(),
                });
            }
        }
        if let Some(digest) = &self.output_digest {
            validate_digest("lineage.step.output_digest", digest)?;
        }
        Ok(())
    }
}

/// Provenance graph for one output observation.
///
/// `steps` are serialized in topological order. `terminal_step_id` must name
/// the final step and every declared root/step must be reachable from it. This
/// prevents a lineage from carrying unrelated provenance that did not actually
/// contribute to the declared output.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EvidenceLineage {
    pub schema_version: u16,
    pub output_observation_id: String,
    pub roots: Vec<LineageRoot>,
    pub steps: Vec<LineageStep>,
    pub terminal_step_id: String,
}

impl EvidenceLineage {
    pub fn new(
        output_observation_id: impl Into<String>,
        roots: Vec<LineageRoot>,
        steps: Vec<LineageStep>,
        terminal_step_id: impl Into<String>,
    ) -> Result<Self, PlanetaryLineageError> {
        let lineage = Self {
            schema_version: PLANETARY_LINEAGE_SCHEMA_VERSION,
            output_observation_id: output_observation_id.into(),
            roots,
            steps,
            terminal_step_id: terminal_step_id.into(),
        };
        lineage.validate()?;
        Ok(lineage)
    }

    pub fn validate(&self) -> Result<(), PlanetaryLineageError> {
        if self.schema_version != PLANETARY_LINEAGE_SCHEMA_VERSION {
            return Err(PlanetaryLineageError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        require_text(
            "lineage.output_observation_id",
            &self.output_observation_id,
            MAX_LINEAGE_ID_BYTES,
        )?;
        require_text(
            "lineage.terminal_step_id",
            &self.terminal_step_id,
            MAX_LINEAGE_ID_BYTES,
        )?;

        if self.roots.is_empty() {
            return Err(PlanetaryLineageError::MissingRoots);
        }
        if self.roots.len() > MAX_LINEAGE_ROOTS {
            return Err(PlanetaryLineageError::TooManyRoots {
                actual: self.roots.len(),
                max: MAX_LINEAGE_ROOTS,
            });
        }
        if self.steps.is_empty() {
            return Err(PlanetaryLineageError::MissingSteps);
        }
        if self.steps.len() > MAX_LINEAGE_STEPS {
            return Err(PlanetaryLineageError::TooManySteps {
                actual: self.steps.len(),
                max: MAX_LINEAGE_STEPS,
            });
        }

        let mut root_ids = HashSet::with_capacity(self.roots.len());
        let mut root_sources = HashSet::with_capacity(self.roots.len());
        for root in &self.roots {
            root.validate(&self.output_observation_id)?;
            if !root_ids.insert(root.id.as_str()) {
                return Err(PlanetaryLineageError::DuplicateRootId(root.id.clone()));
            }
            if !root_sources.insert(&root.source) {
                return Err(PlanetaryLineageError::DuplicateRootSource(root.id.clone()));
            }
        }

        let mut prior_steps: HashSet<&str> = HashSet::with_capacity(self.steps.len());
        let mut all_step_ids: HashSet<&str> = HashSet::with_capacity(self.steps.len());
        for step in &self.steps {
            step.validate_shape()?;
            if !all_step_ids.insert(step.id.as_str()) {
                return Err(PlanetaryLineageError::DuplicateStepId(step.id.clone()));
            }

            for input in &step.inputs {
                match input {
                    LineageRef::Root(root_id) => {
                        if !root_ids.contains(root_id.as_str()) {
                            return Err(PlanetaryLineageError::UnknownRootRef {
                                step_id: step.id.clone(),
                                root_id: root_id.clone(),
                            });
                        }
                    }
                    LineageRef::Step(step_id) => {
                        if step_id == &step.id {
                            return Err(PlanetaryLineageError::SelfStepReference(
                                step.id.clone(),
                            ));
                        }
                        if !prior_steps.contains(step_id.as_str()) {
                            return Err(PlanetaryLineageError::ForwardOrUnknownStepRef {
                                step_id: step.id.clone(),
                                referenced_step_id: step_id.clone(),
                            });
                        }
                    }
                }
            }
            prior_steps.insert(step.id.as_str());
        }

        let last_step = self.steps.last().expect("non-empty steps checked above");
        if last_step.id != self.terminal_step_id {
            return Err(PlanetaryLineageError::TerminalNotLast {
                terminal_step_id: self.terminal_step_id.clone(),
                last_step_id: last_step.id.clone(),
            });
        }

        // Walk backward from the terminal step. Every declared step and root
        // must contribute to the output; unrelated provenance is rejected.
        let mut reachable_steps: HashSet<&str> = HashSet::new();
        let mut used_roots: HashSet<&str> = HashSet::new();
        let mut pending = vec![self.terminal_step_id.as_str()];
        while let Some(step_id) = pending.pop() {
            if !reachable_steps.insert(step_id) {
                continue;
            }
            let step = self
                .steps
                .iter()
                .find(|step| step.id == step_id)
                .ok_or_else(|| PlanetaryLineageError::UnknownTerminalStep(step_id.to_string()))?;
            for input in &step.inputs {
                match input {
                    LineageRef::Root(root_id) => {
                        used_roots.insert(root_id.as_str());
                    }
                    LineageRef::Step(input_step_id) => pending.push(input_step_id.as_str()),
                }
            }
        }

        for step in &self.steps {
            if !reachable_steps.contains(step.id.as_str()) {
                return Err(PlanetaryLineageError::OrphanStep(step.id.clone()));
            }
        }
        for root in &self.roots {
            if !used_roots.contains(root.id.as_str()) {
                return Err(PlanetaryLineageError::UnusedRoot(root.id.clone()));
            }
        }

        Ok(())
    }

    /// True only when every step declares code/config/environment digests.
    /// This is a declaration-completeness check, not proof of deterministic
    /// replay or scientific correctness.
    pub fn has_complete_reproducibility_capsule(&self) -> bool {
        self.steps
            .iter()
            .all(|step| step.producer.has_complete_capsule())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlanetaryLineageError {
    UnsupportedSchemaVersion(u16),
    EmptyField(&'static str),
    FieldTooLong {
        field: &'static str,
        actual: usize,
        max: usize,
    },
    MalformedDigest {
        field: &'static str,
        reason: &'static str,
    },
    InvalidExternalEvidence(String),
    MissingRoots,
    TooManyRoots {
        actual: usize,
        max: usize,
    },
    MissingSteps,
    TooManySteps {
        actual: usize,
        max: usize,
    },
    MissingInputs(String),
    TooManyInputs {
        step_id: String,
        actual: usize,
        max: usize,
    },
    DuplicateRootId(String),
    DuplicateRootSource(String),
    DuplicateStepId(String),
    DuplicateInputRef {
        step_id: String,
        input_id: String,
    },
    UnknownRootRef {
        step_id: String,
        root_id: String,
    },
    SelfStepReference(String),
    ForwardOrUnknownStepRef {
        step_id: String,
        referenced_step_id: String,
    },
    OutputUsedAsInput(String),
    UnknownTerminalStep(String),
    TerminalNotLast {
        terminal_step_id: String,
        last_step_id: String,
    },
    OrphanStep(String),
    UnusedRoot(String),
}

impl fmt::Display for PlanetaryLineageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion(version) => {
                write!(f, "unsupported planetary lineage schema version {version}")
            }
            Self::EmptyField(field) => write!(f, "{field} cannot be empty"),
            Self::FieldTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::MalformedDigest { field, reason } => {
                write!(f, "malformed {field}: {reason}")
            }
            Self::InvalidExternalEvidence(error) => {
                write!(f, "invalid external lineage evidence: {error}")
            }
            Self::MissingRoots => write!(f, "planetary lineage requires at least one root"),
            Self::TooManyRoots { actual, max } => {
                write!(f, "planetary lineage has {actual} roots; maximum is {max}")
            }
            Self::MissingSteps => write!(f, "planetary lineage requires at least one step"),
            Self::TooManySteps { actual, max } => {
                write!(f, "planetary lineage has {actual} steps; maximum is {max}")
            }
            Self::MissingInputs(step_id) => write!(f, "lineage step {step_id} has no inputs"),
            Self::TooManyInputs {
                step_id,
                actual,
                max,
            } => write!(f, "lineage step {step_id} has {actual} inputs; maximum is {max}"),
            Self::DuplicateRootId(id) => write!(f, "duplicate lineage root id {id}"),
            Self::DuplicateRootSource(id) => {
                write!(f, "lineage root {id} aliases a source already declared")
            }
            Self::DuplicateStepId(id) => write!(f, "duplicate lineage step id {id}"),
            Self::DuplicateInputRef { step_id, input_id } => {
                write!(f, "lineage step {step_id} repeats input {input_id}")
            }
            Self::UnknownRootRef { step_id, root_id } => {
                write!(f, "lineage step {step_id} references unknown root {root_id}")
            }
            Self::SelfStepReference(step_id) => {
                write!(f, "lineage step {step_id} cannot reference itself")
            }
            Self::ForwardOrUnknownStepRef {
                step_id,
                referenced_step_id,
            } => write!(
                f,
                "lineage step {step_id} references non-prior step {referenced_step_id}"
            ),
            Self::OutputUsedAsInput(id) => {
                write!(f, "output observation {id} cannot also be a lineage input")
            }
            Self::UnknownTerminalStep(id) => write!(f, "unknown terminal lineage step {id}"),
            Self::TerminalNotLast {
                terminal_step_id,
                last_step_id,
            } => write!(
                f,
                "terminal step {terminal_step_id} must be the final serialized step; last is {last_step_id}"
            ),
            Self::OrphanStep(id) => {
                write!(f, "lineage step {id} does not contribute to the terminal output")
            }
            Self::UnusedRoot(id) => {
                write!(f, "lineage root {id} does not contribute to the terminal output")
            }
        }
    }
}

impl std::error::Error for PlanetaryLineageError {}

fn require_text(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), PlanetaryLineageError> {
    if value.trim().is_empty() {
        return Err(PlanetaryLineageError::EmptyField(field));
    }
    if value.len() > max {
        return Err(PlanetaryLineageError::FieldTooLong {
            field,
            actual: value.len(),
            max,
        });
    }
    Ok(())
}

fn validate_digest(field: &'static str, digest: &str) -> Result<(), PlanetaryLineageError> {
    require_text(field, digest, MAX_LINEAGE_DIGEST_BYTES)?;
    let Some((algorithm, value)) = digest.split_once(':') else {
        return Err(PlanetaryLineageError::MalformedDigest {
            field,
            reason: "digest must be algorithm-qualified",
        });
    };
    if algorithm.trim().is_empty() || value.trim().is_empty() {
        return Err(PlanetaryLineageError::MalformedDigest {
            field,
            reason: "digest algorithm and value must both be non-empty",
        });
    }
    if !algorithm
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'+'))
    {
        return Err(PlanetaryLineageError::MalformedDigest {
            field,
            reason: "digest algorithm contains unsupported characters",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn full_producer(kind: ProducerKind, implementation: &str) -> ProducerIdentity {
        ProducerIdentity {
            kind,
            implementation: implementation.into(),
            version: Some("1.0.0".into()),
            code_digest: Some("sha256:code".into()),
            configuration_digest: Some("sha256:config".into()),
            environment_digest: Some("sha256:env".into()),
        }
    }

    fn valid_lineage() -> EvidenceLineage {
        EvidenceLineage::new(
            "obs:derived:heat-index:1",
            vec![
                LineageRoot {
                    id: "temperature".into(),
                    source: LineageSource::Observation("obs:temperature:1".into()),
                },
                LineageRoot {
                    id: "humidity".into(),
                    source: LineageSource::Observation("obs:humidity:1".into()),
                },
            ],
            vec![
                LineageStep {
                    id: "normalize".into(),
                    operation: "normalize_units".into(),
                    producer: full_producer(
                        ProducerKind::DeterministicTransform,
                        "mycelix://environment/unit-normalizer",
                    ),
                    inputs: vec![
                        LineageRef::Root("temperature".into()),
                        LineageRef::Root("humidity".into()),
                    ],
                    output_digest: Some("blake3:normalized".into()),
                    completed_at: Some(1_788_825_600),
                },
                LineageStep {
                    id: "heat-index".into(),
                    operation: "heat_index".into(),
                    producer: full_producer(
                        ProducerKind::DeterministicTransform,
                        "mycelix://climate/heat-index",
                    ),
                    inputs: vec![LineageRef::Step("normalize".into())],
                    output_digest: Some("blake3:heat-index".into()),
                    completed_at: Some(1_788_825_601),
                },
            ],
            "heat-index",
        )
        .unwrap()
    }

    #[test]
    fn valid_lineage_is_topological_and_complete() {
        let lineage = valid_lineage();
        lineage.validate().unwrap();
        assert!(lineage.has_complete_reproducibility_capsule());
    }

    #[test]
    fn rejects_forward_step_reference() {
        let mut lineage = valid_lineage();
        lineage.steps[0].inputs = vec![LineageRef::Step("heat-index".into())];
        assert!(matches!(
            lineage.validate(),
            Err(PlanetaryLineageError::ForwardOrUnknownStepRef { .. })
        ));
    }

    #[test]
    fn rejects_output_observation_as_root() {
        let mut lineage = valid_lineage();
        lineage.roots[0].source =
            LineageSource::Observation(lineage.output_observation_id.clone());
        assert!(matches!(
            lineage.validate(),
            Err(PlanetaryLineageError::OutputUsedAsInput(_))
        ));
    }

    #[test]
    fn rejects_orphan_steps() {
        let mut lineage = valid_lineage();
        lineage.steps.insert(
            1,
            LineageStep {
                id: "unused".into(),
                operation: "unused_transform".into(),
                producer: full_producer(
                    ProducerKind::DeterministicTransform,
                    "mycelix://test/unused",
                ),
                inputs: vec![LineageRef::Root("temperature".into())],
                output_digest: None,
                completed_at: None,
            },
        );
        assert!(matches!(
            lineage.validate(),
            Err(PlanetaryLineageError::OrphanStep(id)) if id == "unused"
        ));
    }

    #[test]
    fn rejects_unused_roots() {
        let mut lineage = valid_lineage();
        lineage.steps[0].inputs = vec![LineageRef::Root("temperature".into())];
        assert!(matches!(
            lineage.validate(),
            Err(PlanetaryLineageError::UnusedRoot(id)) if id == "humidity"
        ));
    }

    #[test]
    fn rejects_terminal_that_is_not_last() {
        let mut lineage = valid_lineage();
        lineage.terminal_step_id = "normalize".into();
        assert!(matches!(
            lineage.validate(),
            Err(PlanetaryLineageError::TerminalNotLast { .. })
        ));
    }

    #[test]
    fn rejects_malformed_reproducibility_digest() {
        let mut lineage = valid_lineage();
        lineage.steps[0].producer.environment_digest = Some("not-qualified".into());
        assert!(matches!(
            lineage.validate(),
            Err(PlanetaryLineageError::MalformedDigest { .. })
        ));
    }

    #[test]
    fn incomplete_capsule_is_explicit_not_invalid() {
        let mut lineage = valid_lineage();
        lineage.steps[0].producer.environment_digest = None;
        lineage.validate().unwrap();
        assert!(!lineage.has_complete_reproducibility_capsule());
    }

    #[test]
    fn external_evidence_can_be_a_lineage_root() {
        let lineage = EvidenceLineage::new(
            "obs:derived:1",
            vec![LineageRoot {
                id: "source-file".into(),
                source: LineageSource::ExternalEvidence(ExternalEvidenceRef {
                    source_system: "stac:example".into(),
                    resource_id: "collection/item".into(),
                    content_digest: Some("sha256:artifact".into()),
                    retrieved_at: Some(1_788_825_600),
                    license: Some("CC-BY-4.0".into()),
                }),
            }],
            vec![LineageStep {
                id: "ingest".into(),
                operation: "stac_to_observation".into(),
                producer: full_producer(
                    ProducerKind::DeterministicTransform,
                    "mycelix://adapters/stac",
                ),
                inputs: vec![LineageRef::Root("source-file".into())],
                output_digest: None,
                completed_at: None,
            }],
            "ingest",
        )
        .unwrap();
        lineage.validate().unwrap();
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_lineage() {
        let lineage = valid_lineage();
        let encoded = serde_json::to_string(&lineage).unwrap();
        let decoded: EvidenceLineage = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, lineage);
        decoded.validate().unwrap();
    }
}
