// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Auditable civic/institutional decisions over planetary response proposals.
//!
//! A decision record preserves *who decided what, under which declared rule,
//! after considering which immutable evidence and alternatives*. It is still not
//! an execution capability. Selection of an intervention and authorization to
//! actuate that intervention remain separate protocol events.

use crate::{ExternalEvidenceRef, ResponseProposal};
use std::{collections::HashSet, fmt};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub const RESPONSE_DECISION_SCHEMA_VERSION: u16 = 1;
pub const MAX_DECISION_ID_BYTES: usize = 256;
pub const MAX_DECISION_LABEL_BYTES: usize = 256;
pub const MAX_DECISION_MAKERS: usize = 256;
pub const MAX_DECISION_EVIDENCE: usize = 512;
pub const MAX_DECISION_POLICIES: usize = 64;
pub const MAX_DECISION_POSITIONS: usize = 128;

/// Exact proposal representation considered by the decision process.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DecisionProposalRef {
    pub proposal_id: String,
    pub proposal_digest: String,
}

impl DecisionProposalRef {
    pub fn validate(&self) -> Result<(), PlanetaryDecisionError> {
        require_text("decision.proposal_id", &self.proposal_id, MAX_DECISION_ID_BYTES)?;
        validate_digest("decision.proposal_digest", &self.proposal_digest)
    }
}

/// Claimed participant in a decision process.
///
/// A DID/role claim is not proof of authority. Authority qualification and the
/// eventual execution capability are resolved independently.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DecisionMakerClaim {
    pub did: String,
    pub role: String,
    pub constituency: Option<String>,
}

impl DecisionMakerClaim {
    pub fn validate(&self) -> Result<(), PlanetaryDecisionError> {
        require_text("decision.maker.did", &self.did, MAX_DECISION_ID_BYTES)?;
        if !self.did.starts_with("did:") {
            return Err(PlanetaryDecisionError::MalformedDecisionMakerDid);
        }
        require_text("decision.maker.role", &self.role, MAX_DECISION_LABEL_BYTES)?;
        if let Some(constituency) = &self.constituency {
            require_text(
                "decision.maker.constituency",
                constituency,
                MAX_DECISION_LABEL_BYTES,
            )?;
        }
        Ok(())
    }
}

/// Mechanism by which a disposition was reached. This describes process, not
/// legitimacy; legitimacy depends on independently evaluated institutional rules.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum DecisionMechanism {
    Individual,
    DelegatedOfficer,
    Committee,
    DeliberativeBody,
    Vote,
    Consensus,
    EmergencyProcedure,
    Custom(String),
}

impl DecisionMechanism {
    fn validate(&self) -> Result<(), PlanetaryDecisionError> {
        if let Self::Custom(value) = self {
            require_text(
                "decision.mechanism.custom",
                value,
                MAX_DECISION_LABEL_BYTES,
            )?;
        }
        Ok(())
    }
}

/// Immutable artifact considered during deliberation/decision.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DecisionEvidenceRef {
    pub artifact_type: String,
    pub artifact_id: String,
    pub artifact_digest: String,
}

impl DecisionEvidenceRef {
    pub fn validate(&self) -> Result<(), PlanetaryDecisionError> {
        require_text(
            "decision.evidence.artifact_type",
            &self.artifact_type,
            MAX_DECISION_LABEL_BYTES,
        )?;
        require_text(
            "decision.evidence.artifact_id",
            &self.artifact_id,
            MAX_DECISION_ID_BYTES,
        )?;
        validate_digest("decision.evidence.artifact_digest", &self.artifact_digest)
    }
}

/// Decision outcome. Selecting an option records preference/choice but does not
/// itself grant permission for physical or financial execution.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum ResponseDisposition {
    SelectOption { option_id: String },
    RejectAll,
    Defer { reconsider_after: Option<i64> },
    RequestRevision,
}

impl ResponseDisposition {
    fn validate(&self) -> Result<(), PlanetaryDecisionError> {
        if let Self::SelectOption { option_id } = self {
            require_text("decision.option_id", option_id, MAX_DECISION_ID_BYTES)?;
        }
        Ok(())
    }
}

/// Recorded alternate/minority position. Counts are intentionally aggregate;
/// identities can be preserved separately through signed attestations when a
/// process requires named positions.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DecisionPosition {
    pub disposition: ResponseDisposition,
    pub supporters: u32,
    pub rationale_ref: Option<ExternalEvidenceRef>,
}

impl DecisionPosition {
    pub fn validate(&self) -> Result<(), PlanetaryDecisionError> {
        self.disposition.validate()?;
        if let Some(reference) = &self.rationale_ref {
            validate_immutable_external("decision.position.rationale", reference)?;
        }
        Ok(())
    }
}

/// Immutable semantic record of a decision over one response proposal.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ResponseDecisionRecord {
    pub schema_version: u16,
    pub id: String,
    pub proposal: DecisionProposalRef,
    pub decided_at: i64,
    pub mechanism: DecisionMechanism,
    pub decision_makers: Vec<DecisionMakerClaim>,
    /// Digest-bound rules/policies claimed to govern the decision process.
    pub governing_policies: Vec<ExternalEvidenceRef>,
    pub evidence_considered: Vec<DecisionEvidenceRef>,
    pub disposition: ResponseDisposition,
    /// Detailed rationale may remain off-chain, but its exact representation is
    /// digest-bound here when supplied.
    pub rationale_ref: Option<ExternalEvidenceRef>,
    pub alternate_positions: Vec<DecisionPosition>,
}

impl ResponseDecisionRecord {
    pub fn validate(&self) -> Result<(), PlanetaryDecisionError> {
        if self.schema_version != RESPONSE_DECISION_SCHEMA_VERSION {
            return Err(PlanetaryDecisionError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        require_text("decision.id", &self.id, MAX_DECISION_ID_BYTES)?;
        self.proposal.validate()?;
        self.mechanism.validate()?;
        self.disposition.validate()?;

        if self.decision_makers.is_empty() {
            return Err(PlanetaryDecisionError::MissingDecisionMakers);
        }
        if self.decision_makers.len() > MAX_DECISION_MAKERS {
            return Err(PlanetaryDecisionError::TooManyDecisionMakers {
                actual: self.decision_makers.len(),
                max: MAX_DECISION_MAKERS,
            });
        }
        let mut makers = HashSet::with_capacity(self.decision_makers.len());
        for maker in &self.decision_makers {
            maker.validate()?;
            if !makers.insert(maker) {
                return Err(PlanetaryDecisionError::DuplicateDecisionMaker(
                    maker.did.clone(),
                ));
            }
        }

        if self.governing_policies.len() > MAX_DECISION_POLICIES {
            return Err(PlanetaryDecisionError::TooManyPolicies {
                actual: self.governing_policies.len(),
                max: MAX_DECISION_POLICIES,
            });
        }
        let mut policies = HashSet::with_capacity(self.governing_policies.len());
        for policy in &self.governing_policies {
            validate_immutable_external("decision.governing_policy", policy)?;
            if !policies.insert(policy) {
                return Err(PlanetaryDecisionError::DuplicatePolicy);
            }
        }

        if self.evidence_considered.len() > MAX_DECISION_EVIDENCE {
            return Err(PlanetaryDecisionError::TooMuchEvidence {
                actual: self.evidence_considered.len(),
                max: MAX_DECISION_EVIDENCE,
            });
        }
        let mut evidence = HashSet::with_capacity(self.evidence_considered.len());
        for reference in &self.evidence_considered {
            reference.validate()?;
            if !evidence.insert(reference) {
                return Err(PlanetaryDecisionError::DuplicateEvidence(
                    reference.artifact_id.clone(),
                ));
            }
        }

        if let Some(rationale) = &self.rationale_ref {
            validate_immutable_external("decision.rationale", rationale)?;
        }

        if self.alternate_positions.len() > MAX_DECISION_POSITIONS {
            return Err(PlanetaryDecisionError::TooManyAlternatePositions {
                actual: self.alternate_positions.len(),
                max: MAX_DECISION_POSITIONS,
            });
        }
        let mut positions = HashSet::with_capacity(self.alternate_positions.len());
        for position in &self.alternate_positions {
            position.validate()?;
            if &position.disposition == &self.disposition {
                return Err(PlanetaryDecisionError::AlternateDuplicatesFinalDisposition);
            }
            if !positions.insert(position) {
                return Err(PlanetaryDecisionError::DuplicateAlternatePosition);
            }
        }

        Ok(())
    }

    /// Ensure option references are meaningful for the exact proposal object.
    /// Digest recomputation remains outside this dependency-light contract.
    pub fn validate_against_proposal(
        &self,
        proposal: &ResponseProposal,
    ) -> Result<(), PlanetaryDecisionError> {
        self.validate()?;
        proposal
            .validate()
            .map_err(|error| PlanetaryDecisionError::InvalidResponseProposal(error.to_string()))?;
        if self.proposal.proposal_id != proposal.id {
            return Err(PlanetaryDecisionError::ProposalIdMismatch);
        }

        let option_exists = |option_id: &str| proposal.options.iter().any(|o| o.id == option_id);
        if let ResponseDisposition::SelectOption { option_id } = &self.disposition
            && !option_exists(option_id)
        {
            return Err(PlanetaryDecisionError::UnknownSelectedOption(option_id.clone()));
        }
        for position in &self.alternate_positions {
            if let ResponseDisposition::SelectOption { option_id } = &position.disposition
                && !option_exists(option_id)
            {
                return Err(PlanetaryDecisionError::UnknownAlternateOption(
                    option_id.clone(),
                ));
            }
        }
        Ok(())
    }

    /// A decision is an auditable choice artifact, never the execution grant.
    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum PlanetaryDecisionError {
    UnsupportedSchemaVersion(u16),
    EmptyField(&'static str),
    FieldTooLong { field: &'static str, actual: usize, max: usize },
    MalformedDigest { field: &'static str, reason: &'static str },
    MalformedDecisionMakerDid,
    MissingDecisionMakers,
    TooManyDecisionMakers { actual: usize, max: usize },
    DuplicateDecisionMaker(String),
    InvalidExternalReference(String),
    MutableExternalReference { field: &'static str, resource_id: String },
    TooManyPolicies { actual: usize, max: usize },
    DuplicatePolicy,
    TooMuchEvidence { actual: usize, max: usize },
    DuplicateEvidence(String),
    TooManyAlternatePositions { actual: usize, max: usize },
    AlternateDuplicatesFinalDisposition,
    DuplicateAlternatePosition,
    InvalidResponseProposal(String),
    ProposalIdMismatch,
    UnknownSelectedOption(String),
    UnknownAlternateOption(String),
}

impl fmt::Display for PlanetaryDecisionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion(v) => write!(f, "unsupported response-decision schema version {v}"),
            Self::EmptyField(field) => write!(f, "{field} cannot be empty"),
            Self::FieldTooLong { field, actual, max } => write!(f, "{field} is {actual} bytes; maximum is {max}"),
            Self::MalformedDigest { field, reason } => write!(f, "malformed {field}: {reason}"),
            Self::MalformedDecisionMakerDid => write!(f, "decision-maker identity must use a DID"),
            Self::MissingDecisionMakers => write!(f, "decision record requires at least one decision-maker claim"),
            Self::TooManyDecisionMakers { actual, max } => write!(f, "decision record has {actual} makers; maximum is {max}"),
            Self::DuplicateDecisionMaker(did) => write!(f, "decision-maker claim {did} is duplicated"),
            Self::InvalidExternalReference(error) => write!(f, "invalid decision external reference: {error}"),
            Self::MutableExternalReference { field, resource_id } => write!(f, "{field} reference {resource_id} must be content-digest bound"),
            Self::TooManyPolicies { actual, max } => write!(f, "decision record has {actual} policies; maximum is {max}"),
            Self::DuplicatePolicy => write!(f, "decision record contains a duplicate governing policy"),
            Self::TooMuchEvidence { actual, max } => write!(f, "decision record has {actual} evidence refs; maximum is {max}"),
            Self::DuplicateEvidence(id) => write!(f, "decision record repeats evidence artifact {id}"),
            Self::TooManyAlternatePositions { actual, max } => write!(f, "decision record has {actual} alternate positions; maximum is {max}"),
            Self::AlternateDuplicatesFinalDisposition => write!(f, "alternate position cannot duplicate the final disposition"),
            Self::DuplicateAlternatePosition => write!(f, "decision record repeats an alternate position"),
            Self::InvalidResponseProposal(error) => write!(f, "invalid response proposal: {error}"),
            Self::ProposalIdMismatch => write!(f, "decision record references a different response proposal"),
            Self::UnknownSelectedOption(id) => write!(f, "decision selects unknown response option {id}"),
            Self::UnknownAlternateOption(id) => write!(f, "alternate position references unknown response option {id}"),
        }
    }
}

impl std::error::Error for PlanetaryDecisionError {}

fn validate_immutable_external(
    field: &'static str,
    reference: &ExternalEvidenceRef,
) -> Result<(), PlanetaryDecisionError> {
    reference
        .validate()
        .map_err(|error| PlanetaryDecisionError::InvalidExternalReference(error.to_string()))?;
    if reference.content_digest.is_none() {
        return Err(PlanetaryDecisionError::MutableExternalReference {
            field,
            resource_id: reference.resource_id.clone(),
        });
    }
    Ok(())
}

fn require_text(field: &'static str, value: &str, max: usize) -> Result<(), PlanetaryDecisionError> {
    if value.trim().is_empty() {
        return Err(PlanetaryDecisionError::EmptyField(field));
    }
    if value.len() > max {
        return Err(PlanetaryDecisionError::FieldTooLong { field, actual: value.len(), max });
    }
    Ok(())
}

fn validate_digest(field: &'static str, digest: &str) -> Result<(), PlanetaryDecisionError> {
    require_text(field, digest, 256)?;
    let Some((algorithm, value)) = digest.split_once(':') else {
        return Err(PlanetaryDecisionError::MalformedDigest {
            field,
            reason: "digest must be algorithm-qualified",
        });
    };
    if algorithm.trim().is_empty() || value.trim().is_empty() {
        return Err(PlanetaryDecisionError::MalformedDigest {
            field,
            reason: "digest algorithm and value must both be non-empty",
        });
    }
    if !algorithm
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'+'))
    {
        return Err(PlanetaryDecisionError::MalformedDigest {
            field,
            reason: "digest algorithm contains unsupported characters",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AuthorityRequirement, EvidenceClass, GeoPoint, InterventionOption, ProjectedOutcomeRef,
        ProjectedOutcomeRole, ResponseProposalMode, Reversibility, RiskAssessmentMode,
        RiskTriggerRef, SpatialExtent, TemporalExtent,
    };

    fn proposal() -> ResponseProposal {
        ResponseProposal::new(
            "response:heat:1",
            ResponseProposalMode::OperationalRecommendation,
            RiskTriggerRef {
                assessment_id: "risk:heat:1".into(),
                assessment_digest: "sha256:risk".into(),
                risk_output_observation_id: "obs:risk:heat".into(),
                expected_mode: RiskAssessmentMode::Operational,
            },
            100,
            None,
            vec![InterventionOption {
                id: "cooling-centers".into(),
                intervention_type: "open_cooling_centers".into(),
                target: SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
                execution_window: TemporalExtent::new(110, 200).unwrap(),
                projected_outcomes: vec![ProjectedOutcomeRef {
                    observation_id: "obs:projected:1".into(),
                    expected_class: EvidenceClass::Scenario,
                    role: ProjectedOutcomeRole::Benefit,
                }],
                resources: Vec::new(),
                authority_requirements: vec![AuthorityRequirement {
                    domain: "municipal_emergency".into(),
                    action: "open_cooling_center".into(),
                    jurisdiction: Some("Johannesburg".into()),
                    policy_ref: None,
                }],
                reversibility: Reversibility::Reversible,
                assumptions: Vec::new(),
            }],
        )
        .unwrap()
    }

    fn decision() -> ResponseDecisionRecord {
        ResponseDecisionRecord {
            schema_version: RESPONSE_DECISION_SCHEMA_VERSION,
            id: "decision:heat:1".into(),
            proposal: DecisionProposalRef {
                proposal_id: "response:heat:1".into(),
                proposal_digest: "sha256:proposal".into(),
            },
            decided_at: 105,
            mechanism: DecisionMechanism::EmergencyProcedure,
            decision_makers: vec![DecisionMakerClaim {
                did: "did:mycelix:official".into(),
                role: "incident_commander".into(),
                constituency: Some("Johannesburg".into()),
            }],
            governing_policies: Vec::new(),
            evidence_considered: vec![DecisionEvidenceRef {
                artifact_type: "physical_risk_assessment".into(),
                artifact_id: "risk:heat:1".into(),
                artifact_digest: "sha256:risk".into(),
            }],
            disposition: ResponseDisposition::SelectOption {
                option_id: "cooling-centers".into(),
            },
            rationale_ref: None,
            alternate_positions: vec![DecisionPosition {
                disposition: ResponseDisposition::RejectAll,
                supporters: 1,
                rationale_ref: None,
            }],
        }
    }

    #[test]
    fn valid_decision_selects_existing_option_but_is_not_authority() {
        let record = decision();
        record.validate_against_proposal(&proposal()).unwrap();
        assert!(!record.grants_execution_authority());
    }

    #[test]
    fn unknown_selected_option_is_rejected() {
        let mut record = decision();
        record.disposition = ResponseDisposition::SelectOption {
            option_id: "not-an-option".into(),
        };
        assert!(matches!(
            record.validate_against_proposal(&proposal()),
            Err(PlanetaryDecisionError::UnknownSelectedOption(_))
        ));
    }

    #[test]
    fn duplicate_final_position_cannot_be_hidden_as_dissent() {
        let mut record = decision();
        record.alternate_positions[0].disposition = record.disposition.clone();
        assert_eq!(
            record.validate(),
            Err(PlanetaryDecisionError::AlternateDuplicatesFinalDisposition)
        );
    }

    #[test]
    fn decision_maker_did_is_identity_claim_not_bare_label() {
        let mut record = decision();
        record.decision_makers[0].did = "official-1".into();
        assert_eq!(
            record.validate(),
            Err(PlanetaryDecisionError::MalformedDecisionMakerDid)
        );
    }

    #[test]
    fn mutable_governing_policy_is_rejected() {
        let mut record = decision();
        record.governing_policies.push(ExternalEvidenceRef {
            source_system: "municipality".into(),
            resource_id: "emergency-policy/latest".into(),
            content_digest: None,
            retrieved_at: None,
            license: None,
        });
        assert!(matches!(
            record.validate(),
            Err(PlanetaryDecisionError::MutableExternalReference { .. })
        ));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn decision_round_trips() {
        let original = decision();
        let encoded = serde_json::to_string(&original).unwrap();
        let decoded: ResponseDecisionRecord = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, original);
    }
}