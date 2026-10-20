// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Dependency-light scientific-lineage and independence policy vocabulary.
//!
//! Independence is deliberately modeled as a vector over lineage dimensions,
//! never as one canonical scalar score. Whether a particular pattern is
//! acceptable depends on an explicit, named, versioned policy supplied by the
//! scientific context (for example a replication protocol or funder rule).

use crate::{Error, Result};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

const MAX_LABEL_BYTES: usize = 256;
const MAX_LINEAGE_DIMENSIONS: usize = 64;

/// One controlled axis along which two scientific activities or evidence
/// sources may share lineage.
///
/// This first vocabulary is intentionally closed so it can be serialized as a
/// deterministic map key across JSON and other interchange formats. New
/// dimensions should be introduced by a versioned vocabulary update rather
/// than by ambiguous free-form map keys.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LineageDimension {
    Dataset,
    Cohort,
    ParticipantPopulation,
    SampleCollection,
    Method,
    ProtocolRegistration,
    Instrument,
    Facility,
    CalibrationChain,
    Institution,
    Laboratory,
    InvestigatorTeam,
    Software,
    AnalysisPipeline,
    Model,
    UpstreamDatabase,
    ReferenceCorpus,
    Benchmark,
    Material,
    Specimen,
}

/// Observed sharing state for one lineage dimension.
///
/// `Shared` is descriptive, not pejorative. In computational reproduction,
/// shared software or environment may be exactly what the protocol requires.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IndependenceState {
    Independent,
    Shared,
    PartiallyShared,
    Unknown,
    NotApplicable,
}

/// Requirement imposed by a specific scientific policy on one lineage axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IndependenceRequirement {
    /// The observed lineage must be independent on this dimension.
    RequireIndependent,
    /// The observed lineage must intentionally share this dimension.
    RequireShared,
    /// Independent, shared, or partially-shared are all acceptable, but the
    /// lineage still has to be known.
    AllowShared,
    /// This policy does not evaluate this dimension.
    Ignore,
}

/// Observed independence vector. Missing dimensions are explicitly interpreted
/// as [`IndependenceState::Unknown`], never as independent.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(try_from = "IndependenceProfileWire")]
pub struct IndependenceProfile {
    dimensions: BTreeMap<LineageDimension, IndependenceState>,
}

#[derive(Debug, Clone, Deserialize, Default)]
struct IndependenceProfileWire {
    dimensions: BTreeMap<LineageDimension, IndependenceState>,
}

impl TryFrom<IndependenceProfileWire> for IndependenceProfile {
    type Error = Error;

    fn try_from(value: IndependenceProfileWire) -> Result<Self> {
        Self::new(value.dimensions)
    }
}

impl IndependenceProfile {
    pub fn new(dimensions: BTreeMap<LineageDimension, IndependenceState>) -> Result<Self> {
        if dimensions.len() > MAX_LINEAGE_DIMENSIONS {
            return Err(Error::Validation(format!(
                "independence profile cannot exceed {MAX_LINEAGE_DIMENSIONS} dimensions"
            )));
        }
        Ok(Self { dimensions })
    }

    pub fn state(&self, dimension: LineageDimension) -> IndependenceState {
        self.dimensions
            .get(&dimension)
            .copied()
            .unwrap_or(IndependenceState::Unknown)
    }

    pub fn dimensions(
        &self,
    ) -> impl Iterator<Item = (&LineageDimension, &IndependenceState)> {
        self.dimensions.iter()
    }

    pub fn is_empty(&self) -> bool {
        self.dimensions.is_empty()
    }
}

/// Named/versioned policy for deciding which lineage dimensions matter for a
/// particular scientific purpose. There is deliberately no built-in universal
/// "direct replication" or "conceptual replication" policy.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(try_from = "IndependencePolicyWire")]
pub struct IndependencePolicy {
    policy_id: String,
    policy_version: String,
    requirements: BTreeMap<LineageDimension, IndependenceRequirement>,
}

#[derive(Debug, Clone, Deserialize)]
struct IndependencePolicyWire {
    policy_id: String,
    policy_version: String,
    requirements: BTreeMap<LineageDimension, IndependenceRequirement>,
}

impl TryFrom<IndependencePolicyWire> for IndependencePolicy {
    type Error = Error;

    fn try_from(value: IndependencePolicyWire) -> Result<Self> {
        Self::new(value.policy_id, value.policy_version, value.requirements)
    }
}

impl IndependencePolicy {
    pub fn new(
        policy_id: impl Into<String>,
        policy_version: impl Into<String>,
        requirements: BTreeMap<LineageDimension, IndependenceRequirement>,
    ) -> Result<Self> {
        let policy_id = policy_id.into();
        let policy_version = policy_version.into();
        validate_label(&policy_id, "independence policy id", MAX_LABEL_BYTES)?;
        validate_label(
            &policy_version,
            "independence policy version",
            MAX_LABEL_BYTES,
        )?;
        if requirements.is_empty() {
            return Err(Error::Validation(
                "independence policy requires at least one dimension".to_string(),
            ));
        }
        if requirements.len() > MAX_LINEAGE_DIMENSIONS {
            return Err(Error::Validation(format!(
                "independence policy cannot exceed {MAX_LINEAGE_DIMENSIONS} dimensions"
            )));
        }
        if requirements
            .values()
            .all(|requirement| *requirement == IndependenceRequirement::Ignore)
        {
            return Err(Error::Validation(
                "independence policy cannot ignore every dimension".to_string(),
            ));
        }
        Ok(Self {
            policy_id,
            policy_version,
            requirements,
        })
    }

    pub fn policy_id(&self) -> &str {
        &self.policy_id
    }

    pub fn policy_version(&self) -> &str {
        &self.policy_version
    }

    pub fn requirements(
        &self,
    ) -> impl Iterator<Item = (&LineageDimension, &IndependenceRequirement)> {
        self.requirements.iter()
    }

    pub fn evaluate(&self, profile: &IndependenceProfile) -> IndependenceEvaluation {
        let dimensions = self
            .requirements
            .iter()
            .map(|(dimension, requirement)| {
                (
                    *dimension,
                    evaluate_dimension(*requirement, profile.state(*dimension)),
                )
            })
            .collect();

        IndependenceEvaluation {
            policy_id: self.policy_id.clone(),
            policy_version: self.policy_version.clone(),
            dimensions,
        }
    }
}

/// Result of applying one policy requirement to one observed lineage state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DimensionEvaluation {
    Satisfied,
    Unsatisfied,
    /// Required lineage information was not known.
    Unknown,
    /// The observed profile marked the dimension not applicable, but the
    /// policy did not ignore it.
    NotApplicable,
    /// The policy explicitly ignored this dimension.
    NotEvaluated,
}

/// Policy-relative evaluation. This intentionally exposes the vector of
/// dimension outcomes rather than collapsing them into a scalar score.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IndependenceEvaluation {
    policy_id: String,
    policy_version: String,
    dimensions: BTreeMap<LineageDimension, DimensionEvaluation>,
}

impl IndependenceEvaluation {
    pub fn policy_id(&self) -> &str {
        &self.policy_id
    }

    pub fn policy_version(&self) -> &str {
        &self.policy_version
    }

    pub fn dimensions(
        &self,
    ) -> impl Iterator<Item = (&LineageDimension, &DimensionEvaluation)> {
        self.dimensions.iter()
    }

    pub fn outcome(&self, dimension: LineageDimension) -> Option<DimensionEvaluation> {
        self.dimensions.get(&dimension).copied()
    }

    /// Boolean policy result without inventing a scalar independence score.
    /// Ignored dimensions are neutral; unknown/not-applicable/unsatisfied
    /// required dimensions prevent satisfaction.
    pub fn satisfies_policy(&self) -> bool {
        self.dimensions.values().all(|outcome| {
            matches!(
                outcome,
                DimensionEvaluation::Satisfied | DimensionEvaluation::NotEvaluated
            )
        })
    }

    pub fn has_unresolved_dimensions(&self) -> bool {
        self.dimensions.values().any(|outcome| {
            matches!(
                outcome,
                DimensionEvaluation::Unknown | DimensionEvaluation::NotApplicable
            )
        })
    }
}

fn evaluate_dimension(
    requirement: IndependenceRequirement,
    observed: IndependenceState,
) -> DimensionEvaluation {
    if requirement == IndependenceRequirement::Ignore {
        return DimensionEvaluation::NotEvaluated;
    }
    match observed {
        IndependenceState::Unknown => DimensionEvaluation::Unknown,
        IndependenceState::NotApplicable => DimensionEvaluation::NotApplicable,
        IndependenceState::Independent => match requirement {
            IndependenceRequirement::RequireIndependent | IndependenceRequirement::AllowShared => {
                DimensionEvaluation::Satisfied
            }
            IndependenceRequirement::RequireShared => DimensionEvaluation::Unsatisfied,
            IndependenceRequirement::Ignore => unreachable!(),
        },
        IndependenceState::Shared => match requirement {
            IndependenceRequirement::RequireShared | IndependenceRequirement::AllowShared => {
                DimensionEvaluation::Satisfied
            }
            IndependenceRequirement::RequireIndependent => DimensionEvaluation::Unsatisfied,
            IndependenceRequirement::Ignore => unreachable!(),
        },
        IndependenceState::PartiallyShared => match requirement {
            IndependenceRequirement::AllowShared => DimensionEvaluation::Satisfied,
            IndependenceRequirement::RequireIndependent
            | IndependenceRequirement::RequireShared => DimensionEvaluation::Unsatisfied,
            IndependenceRequirement::Ignore => unreachable!(),
        },
    }
}

fn validate_label(value: &str, label: &str, max_bytes: usize) -> Result<()> {
    if value.is_empty() || value.trim() != value {
        return Err(Error::Validation(format!(
            "{label} must be a non-empty canonical string"
        )));
    }
    if value.len() > max_bytes {
        return Err(Error::Validation(format!(
            "{label} cannot exceed {max_bytes} bytes"
        )));
    }
    if value.chars().any(char::is_control) {
        return Err(Error::Validation(format!(
            "{label} cannot contain control characters"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn policy(requirements: &[(LineageDimension, IndependenceRequirement)]) -> IndependencePolicy {
        IndependencePolicy::new(
            "test-policy",
            "1.0.0",
            requirements.iter().copied().collect(),
        )
        .unwrap()
    }

    fn profile(states: &[(LineageDimension, IndependenceState)]) -> IndependenceProfile {
        IndependenceProfile::new(states.iter().copied().collect()).unwrap()
    }

    #[test]
    fn missing_dimension_is_unknown_not_independent() {
        let p = policy(&[(
            LineageDimension::Dataset,
            IndependenceRequirement::RequireIndependent,
        )]);
        let evaluation = p.evaluate(&IndependenceProfile::default());
        assert_eq!(
            evaluation.outcome(LineageDimension::Dataset),
            Some(DimensionEvaluation::Unknown)
        );
        assert!(!evaluation.satisfies_policy());
        assert!(evaluation.has_unresolved_dimensions());
    }

    #[test]
    fn shared_is_acceptable_only_when_policy_says_so() {
        let observed = profile(&[(LineageDimension::Method, IndependenceState::Shared)]);

        let allowed = policy(&[(
            LineageDimension::Method,
            IndependenceRequirement::AllowShared,
        )]);
        assert!(allowed.evaluate(&observed).satisfies_policy());

        let independent = policy(&[(
            LineageDimension::Method,
            IndependenceRequirement::RequireIndependent,
        )]);
        assert!(!independent.evaluate(&observed).satisfies_policy());

        let intentionally_shared = policy(&[(
            LineageDimension::Method,
            IndependenceRequirement::RequireShared,
        )]);
        assert!(intentionally_shared.evaluate(&observed).satisfies_policy());
    }

    #[test]
    fn direct_replication_like_policy_can_require_data_but_allow_method_reuse() {
        let p = policy(&[
            (
                LineageDimension::Dataset,
                IndependenceRequirement::RequireIndependent,
            ),
            (
                LineageDimension::InvestigatorTeam,
                IndependenceRequirement::RequireIndependent,
            ),
            (
                LineageDimension::Method,
                IndependenceRequirement::AllowShared,
            ),
        ]);
        let observed = profile(&[
            (LineageDimension::Dataset, IndependenceState::Independent),
            (
                LineageDimension::InvestigatorTeam,
                IndependenceState::Independent,
            ),
            (LineageDimension::Method, IndependenceState::Shared),
        ]);
        assert!(p.evaluate(&observed).satisfies_policy());
    }

    #[test]
    fn partially_shared_does_not_satisfy_strict_requirement() {
        let observed = profile(&[(
            LineageDimension::Software,
            IndependenceState::PartiallyShared,
        )]);
        for requirement in [
            IndependenceRequirement::RequireIndependent,
            IndependenceRequirement::RequireShared,
        ] {
            let p = policy(&[(LineageDimension::Software, requirement)]);
            assert_eq!(
                p.evaluate(&observed).outcome(LineageDimension::Software),
                Some(DimensionEvaluation::Unsatisfied)
            );
        }
    }

    #[test]
    fn unknown_does_not_satisfy_allow_shared() {
        let p = policy(&[(
            LineageDimension::Method,
            IndependenceRequirement::AllowShared,
        )]);
        let evaluation = p.evaluate(&IndependenceProfile::default());
        assert_eq!(
            evaluation.outcome(LineageDimension::Method),
            Some(DimensionEvaluation::Unknown)
        );
        assert!(!evaluation.satisfies_policy());
    }

    #[test]
    fn not_applicable_requires_explicit_ignore_to_be_neutral() {
        let observed = profile(&[(
            LineageDimension::Specimen,
            IndependenceState::NotApplicable,
        )]);
        let required = policy(&[(
            LineageDimension::Specimen,
            IndependenceRequirement::RequireIndependent,
        )]);
        assert!(!required.evaluate(&observed).satisfies_policy());

        let ignored = policy(&[
            (LineageDimension::Specimen, IndependenceRequirement::Ignore),
            (
                LineageDimension::Dataset,
                IndependenceRequirement::RequireIndependent,
            ),
        ]);
        let observed = profile(&[
            (
                LineageDimension::Specimen,
                IndependenceState::NotApplicable,
            ),
            (LineageDimension::Dataset, IndependenceState::Independent),
        ]);
        assert!(ignored.evaluate(&observed).satisfies_policy());
    }

    #[test]
    fn policy_cannot_be_empty_or_vacuously_ignore_everything() {
        assert!(IndependencePolicy::new("policy", "1", BTreeMap::new()).is_err());
        assert!(IndependencePolicy::new(
            "policy",
            "1",
            BTreeMap::from([(LineageDimension::Dataset, IndependenceRequirement::Ignore)]),
        )
        .is_err());
    }

    #[test]
    fn malformed_policy_cannot_bypass_validation_through_serde() {
        let empty = r#"{"policy_id":"policy","policy_version":"1","requirements":{}}"#;
        assert!(serde_json::from_str::<IndependencePolicy>(empty).is_err());

        let all_ignored = r#"{
            "policy_id":"policy",
            "policy_version":"1",
            "requirements":{"dataset":"ignore"}
        }"#;
        assert!(serde_json::from_str::<IndependencePolicy>(all_ignored).is_err());
    }

    #[test]
    fn policy_and_profile_round_trip_deterministically() {
        let p = policy(&[
            (
                LineageDimension::Dataset,
                IndependenceRequirement::RequireIndependent,
            ),
            (
                LineageDimension::Method,
                IndependenceRequirement::AllowShared,
            ),
        ]);
        let observed = profile(&[
            (LineageDimension::Method, IndependenceState::Shared),
            (LineageDimension::Dataset, IndependenceState::Independent),
        ]);

        let p_json = serde_json::to_string(&p).unwrap();
        let p_round_trip: IndependencePolicy = serde_json::from_str(&p_json).unwrap();
        assert_eq!(p_round_trip, p);

        let profile_json = serde_json::to_string(&observed).unwrap();
        let profile_round_trip: IndependenceProfile = serde_json::from_str(&profile_json).unwrap();
        assert_eq!(profile_round_trip, observed);
        assert!(p_round_trip.evaluate(&profile_round_trip).satisfies_policy());
    }
}
