// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Evidence-bound intervention proposals for planetary risk response.
//!
//! A response proposal is a recommendation artifact, never an execution grant.
//! It can say *what could be done*, *why*, *with which projected outcomes and
//! resources*, and *which authority would be required*. It deliberately cannot
//! carry an approval, capability token, delegated signing key, actuator command,
//! or execution receipt.
//!
//! The intended boundary is:
//!
//! ```text
//! PhysicalRiskAssessment
//!        ↓
//! ResponseProposal          (this module)
//!        ↓
//! governance / policy / human decision
//!        ↓
//! independently verified authority capability
//!        ↓
//! execution
//!        ↓
//! outcome evidence
//! ```
//!
//! This keeps model reasoning and physical authority structurally separate.

use crate::{
    EvidenceClass, ExternalEvidenceRef, Measurement, PhysicalRiskError, RiskAssessmentMode,
    SpatialExtent, TemporalExtent,
};
use std::{collections::HashSet, fmt};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub const RESPONSE_PROPOSAL_SCHEMA_VERSION: u16 = 1;
pub const MAX_RESPONSE_ID_BYTES: usize = 256;
pub const MAX_RESPONSE_LABEL_BYTES: usize = 256;
pub const MAX_RESPONSE_OPTIONS: usize = 64;
pub const MAX_RESPONSE_OUTCOMES: usize = 128;
pub const MAX_RESPONSE_RESOURCES: usize = 128;
pub const MAX_RESPONSE_AUTHORITIES: usize = 64;
pub const MAX_RESPONSE_ASSUMPTIONS: usize = 64;
pub const MAX_JURISDICTION_BYTES: usize = 512;

/// Whether a proposal addresses a live/forecast operational risk or is only a
/// hypothetical comparison exercise.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum ResponseProposalMode {
    OperationalRecommendation,
    ScenarioComparison,
}

/// Immutable reference to the risk assessment that motivated the proposal.
///
/// The digest binds the proposal to an exact serialized risk-assessment
/// representation rather than a mutable identifier alone.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RiskTriggerRef {
    pub assessment_id: String,
    pub assessment_digest: String,
    pub risk_output_observation_id: String,
    pub expected_mode: RiskAssessmentMode,
}

impl RiskTriggerRef {
    pub fn validate(&self) -> Result<(), ResponseProposalError> {
        require_text(
            "response.trigger.assessment_id",
            &self.assessment_id,
            MAX_RESPONSE_ID_BYTES,
        )?;
        require_text(
            "response.trigger.risk_output_observation_id",
            &self.risk_output_observation_id,
            MAX_RESPONSE_ID_BYTES,
        )?;
        validate_digest(
            "response.trigger.assessment_digest",
            &self.assessment_digest,
        )
    }
}

/// Role played by a projected consequence of an intervention.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum ProjectedOutcomeRole {
    Benefit,
    Harm,
    Tradeoff,
    DistributionalEffect,
}

/// Reference to an evidence-bearing prospective consequence.
///
/// Prospective outcomes cannot be raw `Observed` or `Reported` evidence. The
/// referenced observation should normally be paired with PEF-2 lineage so the
/// model/assumptions that produced it remain inspectable.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ProjectedOutcomeRef {
    pub observation_id: String,
    pub expected_class: EvidenceClass,
    pub role: ProjectedOutcomeRole,
}

impl ProjectedOutcomeRef {
    pub fn validate(&self) -> Result<(), ResponseProposalError> {
        require_text(
            "response.outcome.observation_id",
            &self.observation_id,
            MAX_RESPONSE_ID_BYTES,
        )?;
        if matches!(
            self.expected_class,
            EvidenceClass::Observed | EvidenceClass::Reported
        ) {
            return Err(ResponseProposalError::RawProjectedOutcomeClass(
                self.expected_class,
            ));
        }
        Ok(())
    }
}

/// Resource needed to execute one intervention option.
///
/// `availability_observation_id` may point at an evidence-bearing inventory or
/// capacity observation. Its absence means availability is unknown, not zero and
/// not guaranteed.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ResourceRequirement {
    pub resource_type: String,
    pub quantity: Measurement,
    pub availability_observation_id: Option<String>,
    pub substitutable: bool,
}

impl ResourceRequirement {
    pub fn validate(&self) -> Result<(), ResponseProposalError> {
        require_text(
            "response.resource.resource_type",
            &self.resource_type,
            MAX_RESPONSE_LABEL_BYTES,
        )?;
        self.quantity
            .validate()
            .map_err(|error| ResponseProposalError::InvalidResourceQuantity(error.to_string()))?;
        if self.quantity.value < 0.0 {
            return Err(ResponseProposalError::NegativeResourceQuantity {
                resource_type: self.resource_type.clone(),
                value: self.quantity.value,
            });
        }
        if let Some(id) = &self.availability_observation_id {
            require_text(
                "response.resource.availability_observation_id",
                id,
                MAX_RESPONSE_ID_BYTES,
            )?;
        }
        Ok(())
    }
}

/// Describes authority that would be required before execution.
///
/// This is intentionally a *requirement*, not proof that the proposal possesses
/// the authority. A later authority layer must independently resolve and verify
/// the actual grant/capability at execution time.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AuthorityRequirement {
    /// Authority domain, e.g. `municipal_emergency`, `grid_operator`,
    /// `facility_owner`, `individual_self_authority`.
    pub domain: String,
    /// Requested action class, e.g. `open_cooling_center`, `dispatch_bus`,
    /// `shed_load`, `release_funds`.
    pub action: String,
    pub jurisdiction: Option<String>,
    /// Immutable policy/rule that explains why this authority is required.
    /// This is not an authority credential or execution capability.
    pub policy_ref: Option<ExternalEvidenceRef>,
}

impl AuthorityRequirement {
    pub fn validate(&self) -> Result<(), ResponseProposalError> {
        require_text(
            "response.authority.domain",
            &self.domain,
            MAX_RESPONSE_LABEL_BYTES,
        )?;
        require_text(
            "response.authority.action",
            &self.action,
            MAX_RESPONSE_LABEL_BYTES,
        )?;
        if let Some(jurisdiction) = &self.jurisdiction {
            require_text(
                "response.authority.jurisdiction",
                jurisdiction,
                MAX_JURISDICTION_BYTES,
            )?;
        }
        if let Some(policy) = &self.policy_ref {
            policy
                .validate()
                .map_err(|error| ResponseProposalError::InvalidPolicyReference(error.to_string()))?;
            if policy.content_digest.is_none() {
                return Err(ResponseProposalError::MutablePolicyReference(
                    policy.resource_id.clone(),
                ));
            }
        }
        Ok(())
    }
}

/// How readily an intervention can be undone after execution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Reversibility {
    Reversible,
    PartiallyReversible,
    Irreversible,
    Unknown,
}

/// One candidate response option.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct InterventionOption {
    pub id: String,
    pub intervention_type: String,
    pub target: SpatialExtent,
    pub execution_window: TemporalExtent,
    pub projected_outcomes: Vec<ProjectedOutcomeRef>,
    pub resources: Vec<ResourceRequirement>,
    /// Must be non-empty so every option explicitly acknowledges that execution
    /// requires authority from somewhere, including self/owner authority.
    pub authority_requirements: Vec<AuthorityRequirement>,
    pub reversibility: Reversibility,
    /// Immutable methodology/constraint assumptions specific to this option.
    pub assumptions: Vec<ExternalEvidenceRef>,
}

impl InterventionOption {
    pub fn validate(&self) -> Result<(), ResponseProposalError> {
        require_text("response.option.id", &self.id, MAX_RESPONSE_ID_BYTES)?;
        require_text(
            "response.option.intervention_type",
            &self.intervention_type,
            MAX_RESPONSE_LABEL_BYTES,
        )?;
        self.target
            .validate()
            .map_err(|error| ResponseProposalError::InvalidSpatial(error.to_string()))?;
        self.execution_window
            .validate()
            .map_err(|error| ResponseProposalError::InvalidTemporal(error.to_string()))?;

        if self.projected_outcomes.is_empty() {
            return Err(ResponseProposalError::MissingProjectedOutcomes(
                self.id.clone(),
            ));
        }
        if self.projected_outcomes.len() > MAX_RESPONSE_OUTCOMES {
            return Err(ResponseProposalError::TooManyProjectedOutcomes {
                option_id: self.id.clone(),
                actual: self.projected_outcomes.len(),
                max: MAX_RESPONSE_OUTCOMES,
            });
        }
        let mut outcome_ids = HashSet::with_capacity(self.projected_outcomes.len());
        for outcome in &self.projected_outcomes {
            outcome.validate()?;
            if !outcome_ids.insert(outcome.observation_id.as_str()) {
                return Err(ResponseProposalError::DuplicateProjectedOutcome {
                    option_id: self.id.clone(),
                    observation_id: outcome.observation_id.clone(),
                });
            }
        }

        if self.resources.len() > MAX_RESPONSE_RESOURCES {
            return Err(ResponseProposalError::TooManyResources {
                option_id: self.id.clone(),
                actual: self.resources.len(),
                max: MAX_RESPONSE_RESOURCES,
            });
        }
        let mut resource_types = HashSet::with_capacity(self.resources.len());
        for resource in &self.resources {
            resource.validate()?;
            if !resource_types.insert(resource.resource_type.as_str()) {
                return Err(ResponseProposalError::DuplicateResourceType {
                    option_id: self.id.clone(),
                    resource_type: resource.resource_type.clone(),
                });
            }
        }

        if self.authority_requirements.is_empty() {
            return Err(ResponseProposalError::MissingAuthorityRequirements(
                self.id.clone(),
            ));
        }
        if self.authority_requirements.len() > MAX_RESPONSE_AUTHORITIES {
            return Err(ResponseProposalError::TooManyAuthorityRequirements {
                option_id: self.id.clone(),
                actual: self.authority_requirements.len(),
                max: MAX_RESPONSE_AUTHORITIES,
            });
        }
        let mut authorities = HashSet::with_capacity(self.authority_requirements.len());
        for requirement in &self.authority_requirements {
            requirement.validate()?;
            if !authorities.insert(requirement) {
                return Err(ResponseProposalError::DuplicateAuthorityRequirement(
                    self.id.clone(),
                ));
            }
        }

        if self.assumptions.len() > MAX_RESPONSE_ASSUMPTIONS {
            return Err(ResponseProposalError::TooManyAssumptions {
                option_id: self.id.clone(),
                actual: self.assumptions.len(),
                max: MAX_RESPONSE_ASSUMPTIONS,
            });
        }
        let mut assumptions = HashSet::with_capacity(self.assumptions.len());
        for assumption in &self.assumptions {
            assumption
                .validate()
                .map_err(|error| ResponseProposalError::InvalidAssumption(error.to_string()))?;
            if assumption.content_digest.is_none() {
                return Err(ResponseProposalError::MutableAssumptionReference(
                    assumption.resource_id.clone(),
                ));
            }
            if !assumptions.insert(assumption) {
                return Err(ResponseProposalError::DuplicateAssumption(
                    self.id.clone(),
                ));
            }
        }

        Ok(())
    }
}

/// Proposal artifact produced from a specific physical-risk assessment.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ResponseProposal {
    pub schema_version: u16,
    pub id: String,
    pub mode: ResponseProposalMode,
    pub trigger: RiskTriggerRef,
    pub generated_at: i64,
    pub valid_until: Option<i64>,
    pub options: Vec<InterventionOption>,
    /// Optional preferred option selected by the recommender. This remains a
    /// recommendation only; it does not alter the authority requirements.
    pub preferred_option_id: Option<String>,
}

impl ResponseProposal {
    pub fn new(
        id: impl Into<String>,
        mode: ResponseProposalMode,
        trigger: RiskTriggerRef,
        generated_at: i64,
        valid_until: Option<i64>,
        options: Vec<InterventionOption>,
    ) -> Result<Self, ResponseProposalError> {
        let proposal = Self {
            schema_version: RESPONSE_PROPOSAL_SCHEMA_VERSION,
            id: id.into(),
            mode,
            trigger,
            generated_at,
            valid_until,
            options,
            preferred_option_id: None,
        };
        proposal.validate()?;
        Ok(proposal)
    }

    pub fn validate(&self) -> Result<(), ResponseProposalError> {
        if self.schema_version != RESPONSE_PROPOSAL_SCHEMA_VERSION {
            return Err(ResponseProposalError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        require_text("response.id", &self.id, MAX_RESPONSE_ID_BYTES)?;
        self.trigger.validate()?;

        match self.mode {
            ResponseProposalMode::OperationalRecommendation
                if self.trigger.expected_mode != RiskAssessmentMode::Operational =>
            {
                return Err(ResponseProposalError::ModeTriggerMismatch)
            }
            ResponseProposalMode::ScenarioComparison
                if self.trigger.expected_mode != RiskAssessmentMode::Scenario =>
            {
                return Err(ResponseProposalError::ModeTriggerMismatch)
            }
            _ => {}
        }

        if let Some(valid_until) = self.valid_until
            && valid_until < self.generated_at
        {
            return Err(ResponseProposalError::ExpiryBeforeGeneration {
                generated_at: self.generated_at,
                valid_until,
            });
        }

        if self.options.is_empty() {
            return Err(ResponseProposalError::MissingOptions);
        }
        if self.options.len() > MAX_RESPONSE_OPTIONS {
            return Err(ResponseProposalError::TooManyOptions {
                actual: self.options.len(),
                max: MAX_RESPONSE_OPTIONS,
            });
        }
        let mut option_ids = HashSet::with_capacity(self.options.len());
        for option in &self.options {
            option.validate()?;
            if !option_ids.insert(option.id.as_str()) {
                return Err(ResponseProposalError::DuplicateOptionId(
                    option.id.clone(),
                ));
            }
        }

        if let Some(preferred) = &self.preferred_option_id {
            require_text(
                "response.preferred_option_id",
                preferred,
                MAX_RESPONSE_ID_BYTES,
            )?;
            if !option_ids.contains(preferred.as_str()) {
                return Err(ResponseProposalError::UnknownPreferredOption(
                    preferred.clone(),
                ));
            }
        }

        Ok(())
    }

    pub fn is_expired_at(&self, unix_seconds: i64) -> bool {
        self.valid_until
            .is_some_and(|valid_until| unix_seconds > valid_until)
    }

    /// This is intentionally always false. A proposal can describe required
    /// authority but never contains execution authority itself.
    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ResponseProposalError {
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
    InvalidSpatial(String),
    InvalidTemporal(String),
    InvalidResourceQuantity(String),
    NegativeResourceQuantity {
        resource_type: String,
        value: f64,
    },
    InvalidPolicyReference(String),
    MutablePolicyReference(String),
    InvalidAssumption(String),
    MutableAssumptionReference(String),
    RawProjectedOutcomeClass(EvidenceClass),
    MissingProjectedOutcomes(String),
    TooManyProjectedOutcomes {
        option_id: String,
        actual: usize,
        max: usize,
    },
    DuplicateProjectedOutcome {
        option_id: String,
        observation_id: String,
    },
    TooManyResources {
        option_id: String,
        actual: usize,
        max: usize,
    },
    DuplicateResourceType {
        option_id: String,
        resource_type: String,
    },
    MissingAuthorityRequirements(String),
    TooManyAuthorityRequirements {
        option_id: String,
        actual: usize,
        max: usize,
    },
    DuplicateAuthorityRequirement(String),
    TooManyAssumptions {
        option_id: String,
        actual: usize,
        max: usize,
    },
    DuplicateAssumption(String),
    ModeTriggerMismatch,
    ExpiryBeforeGeneration {
        generated_at: i64,
        valid_until: i64,
    },
    MissingOptions,
    TooManyOptions {
        actual: usize,
        max: usize,
    },
    DuplicateOptionId(String),
    UnknownPreferredOption(String),
}

impl fmt::Display for ResponseProposalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion(version) => {
                write!(f, "unsupported response-proposal schema version {version}")
            }
            Self::EmptyField(field) => write!(f, "{field} cannot be empty"),
            Self::FieldTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::MalformedDigest { field, reason } => {
                write!(f, "malformed {field}: {reason}")
            }
            Self::InvalidSpatial(error) => write!(f, "invalid response spatial extent: {error}"),
            Self::InvalidTemporal(error) => write!(f, "invalid response time extent: {error}"),
            Self::InvalidResourceQuantity(error) => write!(f, "invalid resource quantity: {error}"),
            Self::NegativeResourceQuantity {
                resource_type,
                value,
            } => write!(f, "resource {resource_type} cannot require negative quantity {value}"),
            Self::InvalidPolicyReference(error) => write!(f, "invalid authority policy reference: {error}"),
            Self::MutablePolicyReference(resource) => write!(f, "authority policy {resource} must be content-digest bound"),
            Self::InvalidAssumption(error) => write!(f, "invalid intervention assumption: {error}"),
            Self::MutableAssumptionReference(resource) => write!(f, "intervention assumption {resource} must be content-digest bound"),
            Self::RawProjectedOutcomeClass(class) => write!(f, "projected intervention outcome cannot claim raw evidence class {class:?}"),
            Self::MissingProjectedOutcomes(option) => write!(f, "intervention option {option} requires at least one projected outcome"),
            Self::TooManyProjectedOutcomes { option_id, actual, max } => write!(f, "intervention option {option_id} has {actual} outcomes; maximum is {max}"),
            Self::DuplicateProjectedOutcome { option_id, observation_id } => write!(f, "intervention option {option_id} repeats projected outcome {observation_id}"),
            Self::TooManyResources { option_id, actual, max } => write!(f, "intervention option {option_id} has {actual} resources; maximum is {max}"),
            Self::DuplicateResourceType { option_id, resource_type } => write!(f, "intervention option {option_id} repeats resource type {resource_type}; combine it into one requirement"),
            Self::MissingAuthorityRequirements(option) => write!(f, "intervention option {option} must declare required execution authority"),
            Self::TooManyAuthorityRequirements { option_id, actual, max } => write!(f, "intervention option {option_id} has {actual} authority requirements; maximum is {max}"),
            Self::DuplicateAuthorityRequirement(option) => write!(f, "intervention option {option} has duplicate authority requirements"),
            Self::TooManyAssumptions { option_id, actual, max } => write!(f, "intervention option {option_id} has {actual} assumptions; maximum is {max}"),
            Self::DuplicateAssumption(option) => write!(f, "intervention option {option} has duplicate assumptions"),
            Self::ModeTriggerMismatch => write!(f, "response proposal mode does not match its referenced risk assessment mode"),
            Self::ExpiryBeforeGeneration { generated_at, valid_until } => write!(f, "response proposal expiry {valid_until} precedes generation time {generated_at}"),
            Self::MissingOptions => write!(f, "response proposal requires at least one intervention option"),
            Self::TooManyOptions { actual, max } => write!(f, "response proposal has {actual} options; maximum is {max}"),
            Self::DuplicateOptionId(id) => write!(f, "response proposal repeats option ID {id}"),
            Self::UnknownPreferredOption(id) => write!(f, "preferred response option {id} is not present in the proposal"),
        }
    }
}

impl std::error::Error for ResponseProposalError {}

fn require_text(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), ResponseProposalError> {
    if value.trim().is_empty() {
        return Err(ResponseProposalError::EmptyField(field));
    }
    if value.len() > max {
        return Err(ResponseProposalError::FieldTooLong {
            field,
            actual: value.len(),
            max,
        });
    }
    Ok(())
}

fn validate_digest(field: &'static str, digest: &str) -> Result<(), ResponseProposalError> {
    require_text(field, digest, 256)?;
    let Some((algorithm, value)) = digest.split_once(':') else {
        return Err(ResponseProposalError::MalformedDigest {
            field,
            reason: "digest must be algorithm-qualified",
        });
    };
    if algorithm.trim().is_empty() || value.trim().is_empty() {
        return Err(ResponseProposalError::MalformedDigest {
            field,
            reason: "digest algorithm and value must both be non-empty",
        });
    }
    if !algorithm
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'+'))
    {
        return Err(ResponseProposalError::MalformedDigest {
            field,
            reason: "digest algorithm contains unsupported characters",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{GeoPoint, RiskInputRef, RiskEvidenceRole, RiskOutputRef};

    fn trigger(mode: RiskAssessmentMode) -> RiskTriggerRef {
        RiskTriggerRef {
            assessment_id: "risk:jhb:heat:1".into(),
            assessment_digest: "sha256:risk-assessment".into(),
            risk_output_observation_id: "obs:risk:heat-mortality".into(),
            expected_mode: mode,
        }
    }

    fn option(outcome_class: EvidenceClass) -> InterventionOption {
        InterventionOption {
            id: "cooling-centers".into(),
            intervention_type: "open_cooling_centers".into(),
            target: SpatialExtent::Point(GeoPoint::new(-26.2041, 28.0473).unwrap()),
            execution_window: TemporalExtent::new(1_788_912_000, 1_788_933_600).unwrap(),
            projected_outcomes: vec![ProjectedOutcomeRef {
                observation_id: "obs:projected:heat-mortality:cooling".into(),
                expected_class: outcome_class,
                role: ProjectedOutcomeRole::Benefit,
            }],
            resources: vec![ResourceRequirement {
                resource_type: "cooling_center_capacity".into(),
                quantity: Measurement::new(2_000.0, "person").unwrap(),
                availability_observation_id: Some("obs:available:cooling-capacity".into()),
                substitutable: false,
            }],
            authority_requirements: vec![AuthorityRequirement {
                domain: "municipal_emergency".into(),
                action: "open_cooling_center".into(),
                jurisdiction: Some("Johannesburg".into()),
                policy_ref: None,
            }],
            reversibility: Reversibility::Reversible,
            assumptions: Vec::new(),
        }
    }

    fn operational() -> ResponseProposal {
        ResponseProposal::new(
            "response:jhb:heat:1",
            ResponseProposalMode::OperationalRecommendation,
            trigger(RiskAssessmentMode::Operational),
            1_788_900_000,
            Some(1_788_933_600),
            vec![option(EvidenceClass::Scenario)],
        )
        .unwrap()
    }

    #[test]
    fn operational_recommendation_is_not_execution_authority() {
        let proposal = operational();
        proposal.validate().unwrap();
        assert!(!proposal.grants_execution_authority());
        assert!(!proposal.options[0].authority_requirements.is_empty());
    }

    #[test]
    fn prospective_outcome_cannot_claim_observed() {
        let mut proposal = operational();
        proposal.options[0].projected_outcomes[0].expected_class = EvidenceClass::Observed;
        assert_eq!(
            proposal.validate(),
            Err(ResponseProposalError::RawProjectedOutcomeClass(
                EvidenceClass::Observed
            ))
        );
    }

    #[test]
    fn every_option_requires_explicit_authority_boundary() {
        let mut proposal = operational();
        proposal.options[0].authority_requirements.clear();
        assert!(matches!(
            proposal.validate(),
            Err(ResponseProposalError::MissingAuthorityRequirements(_))
        ));
    }

    #[test]
    fn response_mode_must_match_risk_mode() {
        let mut proposal = operational();
        proposal.trigger.expected_mode = RiskAssessmentMode::Scenario;
        assert_eq!(
            proposal.validate(),
            Err(ResponseProposalError::ModeTriggerMismatch)
        );
    }

    #[test]
    fn preferred_option_must_exist() {
        let mut proposal = operational();
        proposal.preferred_option_id = Some("not-there".into());
        assert!(matches!(
            proposal.validate(),
            Err(ResponseProposalError::UnknownPreferredOption(_))
        ));
    }

    #[test]
    fn negative_resource_requirements_are_rejected() {
        let mut proposal = operational();
        proposal.options[0].resources[0].quantity.value = -1.0;
        assert!(matches!(
            proposal.validate(),
            Err(ResponseProposalError::NegativeResourceQuantity { .. })
        ));
    }

    #[test]
    fn mutable_policy_refs_are_rejected() {
        let mut proposal = operational();
        proposal.options[0].authority_requirements[0].policy_ref = Some(ExternalEvidenceRef {
            source_system: "municipality".into(),
            resource_id: "policy/latest".into(),
            content_digest: None,
            retrieved_at: None,
            license: None,
        });
        assert!(matches!(
            proposal.validate(),
            Err(ResponseProposalError::MutablePolicyReference(_))
        ));
    }

    #[test]
    fn scenario_comparison_stays_distinct_from_operational_recommendation() {
        let proposal = ResponseProposal::new(
            "response:scenario:1",
            ResponseProposalMode::ScenarioComparison,
            trigger(RiskAssessmentMode::Scenario),
            1_788_900_000,
            None,
            vec![option(EvidenceClass::Scenario)],
        )
        .unwrap();
        proposal.validate().unwrap();
        assert_eq!(proposal.mode, ResponseProposalMode::ScenarioComparison);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_authority_requirements() {
        let proposal = operational();
        let encoded = serde_json::to_string(&proposal).unwrap();
        let decoded: ResponseProposal = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, proposal);
    }

    // Keep these imported risk types exercised together so future API changes
    // make the intended response/risk composition visible at compile time.
    #[test]
    fn risk_reference_roles_remain_composable_with_response_trigger() {
        let input = RiskInputRef {
            observation_id: "obs:hazard".into(),
            role: RiskEvidenceRole::Hazard,
            expected_class: EvidenceClass::Forecast,
        };
        let output = RiskOutputRef {
            observation_id: "obs:risk".into(),
            expected_class: EvidenceClass::Inferred,
        };
        assert_eq!(input.role, RiskEvidenceRole::Hazard);
        assert_eq!(output.expected_class, EvidenceClass::Inferred);
    }
}
