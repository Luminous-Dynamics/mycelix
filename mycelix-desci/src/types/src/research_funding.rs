// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Dependency-light research-funding semantics for Mycelix-DeSci.
//!
//! This module deliberately does not implement governance, treasury, payment,
//! settlement, or resource scheduling. It gives the research layer typed
//! identities and source-qualified references so that a scientific project can
//! describe its relationship to funding without silently acquiring authority
//! owned by Mycelix Governance, Mycelix Finance, or an external funder.
//!
//! Core distinctions:
//! - opportunity != application
//! - application != review
//! - review != decision
//! - decision != award
//! - award != allocation
//! - allocation != payment/settlement
//! - governance record != finance settlement proof
//! - scientific milestone completion != desired scientific result

use crate::{Error, Result};
use serde::{Deserialize, Deserializer, Serialize};
use std::fmt;

const MAX_ID_BYTES: usize = 512;
const MAX_LABEL_BYTES: usize = 256;
const MAX_OTHER_KIND_BYTES: usize = 128;

fn validate_canonical_text(value: &str, label: &str, max_bytes: usize) -> Result<()> {
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

macro_rules! funding_id_type {
    ($name:ident, $label:literal) => {
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
        #[serde(transparent)]
        pub struct $name(String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self> {
                let value = value.into();
                validate_canonical_text(&value, $label, MAX_ID_BYTES)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(&self.0)
            }
        }

        impl<'de> Deserialize<'de> for $name {
            fn deserialize<D>(deserializer: D) -> core::result::Result<Self, D::Error>
            where
                D: Deserializer<'de>,
            {
                let value = String::deserialize(deserializer)?;
                Self::new(value).map_err(serde::de::Error::custom)
            }
        }
    };
}

funding_id_type!(FundingOpportunityId, "funding opportunity id");
funding_id_type!(FundingApplicationId, "funding application id");
funding_id_type!(FundingReviewId, "funding review id");
funding_id_type!(FundingDecisionId, "funding decision id");
funding_id_type!(FundingAwardId, "funding award id");

/// Exact immutable application-version reference.
///
/// `content_id` is an opaque immutable identity supplied by the application
/// authority (for example a content hash or version object identifier). This
/// type does not assert that the version was submitted, reviewed, or accepted.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub struct FundingApplicationVersionRef {
    application_id: FundingApplicationId,
    version: u32,
    content_id: String,
}

#[derive(Debug, Deserialize)]
struct FundingApplicationVersionWire {
    application_id: FundingApplicationId,
    version: u32,
    content_id: String,
}

impl FundingApplicationVersionRef {
    pub fn new(
        application_id: FundingApplicationId,
        version: u32,
        content_id: impl Into<String>,
    ) -> Result<Self> {
        if version == 0 {
            return Err(Error::Validation(
                "funding application version must be at least one".to_string(),
            ));
        }
        let content_id = content_id.into();
        validate_canonical_text(
            &content_id,
            "funding application version content id",
            MAX_ID_BYTES,
        )?;
        Ok(Self {
            application_id,
            version,
            content_id,
        })
    }

    pub fn application_id(&self) -> &FundingApplicationId {
        &self.application_id
    }

    pub fn version(&self) -> u32 {
        self.version
    }

    pub fn content_id(&self) -> &str {
        &self.content_id
    }
}

impl<'de> Deserialize<'de> for FundingApplicationVersionRef {
    fn deserialize<D>(deserializer: D) -> core::result::Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = FundingApplicationVersionWire::deserialize(deserializer)?;
        Self::new(wire.application_id, wire.version, wire.content_id)
            .map_err(serde::de::Error::custom)
    }
}

/// Source-qualified reference to an object owned by another authority.
///
/// A valid reference says only which authority/object is being referred to.
/// It does not prove approval, current state, availability, authorization,
/// payment, or settlement.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub struct FundingAuthorityRef {
    authority: String,
    object_id: String,
}

#[derive(Debug, Deserialize)]
struct FundingAuthorityRefWire {
    authority: String,
    object_id: String,
}

impl FundingAuthorityRef {
    pub fn new(authority: impl Into<String>, object_id: impl Into<String>) -> Result<Self> {
        let authority = authority.into();
        let object_id = object_id.into();
        validate_canonical_text(&authority, "funding authority", MAX_LABEL_BYTES)?;
        validate_canonical_text(&object_id, "funding authority object id", MAX_ID_BYTES)?;
        Ok(Self {
            authority,
            object_id,
        })
    }

    pub fn authority(&self) -> &str {
        &self.authority
    }

    pub fn object_id(&self) -> &str {
        &self.object_id
    }
}

impl<'de> Deserialize<'de> for FundingAuthorityRef {
    fn deserialize<D>(deserializer: D) -> core::result::Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = FundingAuthorityRefWire::deserialize(deserializer)?;
        Self::new(wire.authority, wire.object_id).map_err(serde::de::Error::custom)
    }
}

/// Resource granted or committed by an award.
///
/// The kind is descriptive only. In particular, `MonetaryFunding` does not
/// carry an amount or settlement state; those remain authoritative in Finance
/// or the external monetary authority referenced by [`AwardResourceRef`].
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AwardResourceKind {
    MonetaryFunding,
    ComputeTime,
    FacilityTime,
    InstrumentTime,
    Materials,
    DatasetAccess,
    PersonnelSupport,
    TravelLogistics,
    Other(String),
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum AwardResourceKindWire {
    MonetaryFunding,
    ComputeTime,
    FacilityTime,
    InstrumentTime,
    Materials,
    DatasetAccess,
    PersonnelSupport,
    TravelLogistics,
    Other(String),
}

impl AwardResourceKind {
    fn validate(&self) -> Result<()> {
        if let Self::Other(value) = self {
            validate_canonical_text(value, "custom award resource kind", MAX_OTHER_KIND_BYTES)?;
        }
        Ok(())
    }
}

impl<'de> Deserialize<'de> for AwardResourceKind {
    fn deserialize<D>(deserializer: D) -> core::result::Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = AwardResourceKindWire::deserialize(deserializer)?;
        let kind = match wire {
            AwardResourceKindWire::MonetaryFunding => Self::MonetaryFunding,
            AwardResourceKindWire::ComputeTime => Self::ComputeTime,
            AwardResourceKindWire::FacilityTime => Self::FacilityTime,
            AwardResourceKindWire::InstrumentTime => Self::InstrumentTime,
            AwardResourceKindWire::Materials => Self::Materials,
            AwardResourceKindWire::DatasetAccess => Self::DatasetAccess,
            AwardResourceKindWire::PersonnelSupport => Self::PersonnelSupport,
            AwardResourceKindWire::TravelLogistics => Self::TravelLogistics,
            AwardResourceKindWire::Other(value) => Self::Other(value),
        };
        kind.validate().map_err(serde::de::Error::custom)?;
        Ok(kind)
    }
}

/// Reference to one resource commitment/allocation owned by its source
/// authority. Presence of this reference does not assert that the resource was
/// delivered or, for money, that a payment settled.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct AwardResourceRef {
    kind: AwardResourceKind,
    authority_record: FundingAuthorityRef,
}

impl AwardResourceRef {
    pub fn new(kind: AwardResourceKind, authority_record: FundingAuthorityRef) -> Result<Self> {
        kind.validate()?;
        Ok(Self {
            kind,
            authority_record,
        })
    }

    pub fn kind(&self) -> &AwardResourceKind {
        &self.kind
    }

    pub fn authority_record(&self) -> &FundingAuthorityRef {
        &self.authority_record
    }
}

/// Outcome-neutral kind of scientific milestone.
///
/// Milestones describe trustworthy execution or delivery. They deliberately do
/// not encode whether a hypothesis was supported, an effect was significant,
/// or a replication produced a preferred direction.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ScientificMilestoneKind {
    ProtocolVersionSealed,
    RegisteredExecutionCompleted,
    DatasetDeposited,
    AnalysisWorkflowArchived,
    ReplicationAttemptCompleted,
    ReproducibilityCapsulePublished,
    ReportDelivered,
    OtherDelivery(String),
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ScientificMilestoneKindWire {
    ProtocolVersionSealed,
    RegisteredExecutionCompleted,
    DatasetDeposited,
    AnalysisWorkflowArchived,
    ReplicationAttemptCompleted,
    ReproducibilityCapsulePublished,
    ReportDelivered,
    OtherDelivery(String),
}

impl ScientificMilestoneKind {
    pub fn validate(&self) -> Result<()> {
        if let Self::OtherDelivery(value) = self {
            validate_canonical_text(value, "custom scientific milestone kind", MAX_OTHER_KIND_BYTES)?;
            reject_outcome_semantics(value)?;
        }
        Ok(())
    }

    /// Milestone kinds in this module are execution/delivery semantics only.
    pub const fn is_outcome_neutral(&self) -> bool {
        true
    }
}

impl<'de> Deserialize<'de> for ScientificMilestoneKind {
    fn deserialize<D>(deserializer: D) -> core::result::Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = ScientificMilestoneKindWire::deserialize(deserializer)?;
        let kind = match wire {
            ScientificMilestoneKindWire::ProtocolVersionSealed => Self::ProtocolVersionSealed,
            ScientificMilestoneKindWire::RegisteredExecutionCompleted => {
                Self::RegisteredExecutionCompleted
            }
            ScientificMilestoneKindWire::DatasetDeposited => Self::DatasetDeposited,
            ScientificMilestoneKindWire::AnalysisWorkflowArchived => Self::AnalysisWorkflowArchived,
            ScientificMilestoneKindWire::ReplicationAttemptCompleted => {
                Self::ReplicationAttemptCompleted
            }
            ScientificMilestoneKindWire::ReproducibilityCapsulePublished => {
                Self::ReproducibilityCapsulePublished
            }
            ScientificMilestoneKindWire::ReportDelivered => Self::ReportDelivered,
            ScientificMilestoneKindWire::OtherDelivery(value) => Self::OtherDelivery(value),
        };
        kind.validate().map_err(serde::de::Error::custom)?;
        Ok(kind)
    }
}

/// An outcome-neutral milestone criterion attached to a research award.
///
/// The criterion has no free-form success expression: its typed `kind` is the
/// success semantics. `criterion_id` is only a stable local identifier.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub struct ScientificMilestoneCriterion {
    criterion_id: String,
    kind: ScientificMilestoneKind,
}

#[derive(Debug, Deserialize)]
struct ScientificMilestoneCriterionWire {
    criterion_id: String,
    kind: ScientificMilestoneKind,
}

impl ScientificMilestoneCriterion {
    pub fn new(
        criterion_id: impl Into<String>,
        kind: ScientificMilestoneKind,
    ) -> Result<Self> {
        let criterion_id = criterion_id.into();
        validate_canonical_text(&criterion_id, "scientific milestone criterion id", MAX_ID_BYTES)?;
        kind.validate()?;
        Ok(Self { criterion_id, kind })
    }

    pub fn criterion_id(&self) -> &str {
        &self.criterion_id
    }

    pub fn kind(&self) -> &ScientificMilestoneKind {
        &self.kind
    }
}

impl<'de> Deserialize<'de> for ScientificMilestoneCriterion {
    fn deserialize<D>(deserializer: D) -> core::result::Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = ScientificMilestoneCriterionWire::deserialize(deserializer)?;
        Self::new(wire.criterion_id, wire.kind).map_err(serde::de::Error::custom)
    }
}

fn reject_outcome_semantics(value: &str) -> Result<()> {
    let normalized = value
        .chars()
        .map(|ch| {
            if ch.is_ascii_alphanumeric() {
                ch.to_ascii_lowercase()
            } else {
                '_'
            }
        })
        .collect::<String>()
        .split('_')
        .filter(|part| !part.is_empty())
        .collect::<Vec<_>>()
        .join("_");

    let forbidden = [
        "significant_result",
        "statistically_significant",
        "positive_result",
        "negative_result",
        "null_result",
        "supports_hypothesis",
        "hypothesis_supported",
        "refutes_hypothesis",
        "hypothesis_refuted",
        "target_effect_size",
        "desired_effect_size",
        "successful_replication",
        "desired_outcome",
    ];

    if forbidden.iter().any(|term| normalized.contains(term)) {
        return Err(Error::Validation(
            "scientific milestone cannot require a desired scientific outcome".to_string(),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lifecycle_identifiers_validate_independently() {
        let opportunity = FundingOpportunityId::new("opportunity:nih:r01:2026").unwrap();
        let application = FundingApplicationId::new("application:local:123").unwrap();
        let review = FundingReviewId::new("review:local:456").unwrap();
        let decision = FundingDecisionId::new("decision:governance:789").unwrap();
        let award = FundingAwardId::new("award:nih:abc").unwrap();

        assert_eq!(opportunity.as_str(), "opportunity:nih:r01:2026");
        assert_eq!(application.as_str(), "application:local:123");
        assert_eq!(review.as_str(), "review:local:456");
        assert_eq!(decision.as_str(), "decision:governance:789");
        assert_eq!(award.as_str(), "award:nih:abc");
    }

    #[test]
    fn application_version_binds_version_and_immutable_content_identity() {
        let application = FundingApplicationId::new("application:local:123").unwrap();
        let version = FundingApplicationVersionRef::new(application, 2, "blake3:deadbeef").unwrap();
        assert_eq!(version.version(), 2);
        assert_eq!(version.content_id(), "blake3:deadbeef");
        assert!(FundingApplicationVersionRef::new(
            FundingApplicationId::new("application:local:123").unwrap(),
            0,
            "blake3:deadbeef",
        )
        .is_err());
    }

    #[test]
    fn authority_reference_does_not_encode_settlement_state() {
        let reference = FundingAuthorityRef::new(
            "mycelix-finance:treasury",
            "allocation:research:001",
        )
        .unwrap();
        let resource = AwardResourceRef::new(AwardResourceKind::MonetaryFunding, reference).unwrap();
        assert_eq!(resource.kind(), &AwardResourceKind::MonetaryFunding);
        assert_eq!(
            resource.authority_record().authority(),
            "mycelix-finance:treasury"
        );
    }

    #[test]
    fn award_can_reference_non_monetary_resources() {
        let compute = AwardResourceRef::new(
            AwardResourceKind::ComputeTime,
            FundingAuthorityRef::new("hpc.example.org", "allocation:gpu:42").unwrap(),
        )
        .unwrap();
        assert_eq!(compute.kind(), &AwardResourceKind::ComputeTime);
    }

    #[test]
    fn milestone_kinds_are_outcome_neutral() {
        let kinds = [
            ScientificMilestoneKind::ProtocolVersionSealed,
            ScientificMilestoneKind::RegisteredExecutionCompleted,
            ScientificMilestoneKind::DatasetDeposited,
            ScientificMilestoneKind::AnalysisWorkflowArchived,
            ScientificMilestoneKind::ReplicationAttemptCompleted,
            ScientificMilestoneKind::ReproducibilityCapsulePublished,
            ScientificMilestoneKind::ReportDelivered,
        ];
        assert!(kinds.iter().all(ScientificMilestoneKind::is_outcome_neutral));
    }

    #[test]
    fn custom_milestone_rejects_desired_scientific_outcomes() {
        for forbidden in [
            "statistically significant result",
            "positive-result",
            "negative_result",
            "null result",
            "supports hypothesis",
            "target effect size achieved",
            "successful replication",
            "desired outcome",
        ] {
            assert!(ScientificMilestoneKind::OtherDelivery(forbidden.to_string())
                .validate()
                .is_err());
        }
    }

    #[test]
    fn custom_milestone_accepts_delivery_semantics() {
        let criterion = ScientificMilestoneCriterion::new(
            "milestone:archive-calibration",
            ScientificMilestoneKind::OtherDelivery(
                "calibration_chain_archived".to_string(),
            ),
        )
        .unwrap();
        assert_eq!(criterion.criterion_id(), "milestone:archive-calibration");
    }

    #[test]
    fn invalid_wire_data_cannot_bypass_validation() {
        let invalid_id = r#"  application:bad  "#;
        assert!(serde_json::from_str::<FundingApplicationId>(invalid_id).is_err());

        let invalid_version = r#"{
            "application_id":"application:local:123",
            "version":0,
            "content_id":"blake3:deadbeef"
        }"#;
        assert!(serde_json::from_str::<FundingApplicationVersionRef>(invalid_version).is_err());

        let invalid_milestone = r#"{
            "criterion_id":"milestone:bad",
            "kind":{"other_delivery":"supports hypothesis"}
        }"#;
        assert!(serde_json::from_str::<ScientificMilestoneCriterion>(invalid_milestone).is_err());
    }

    #[test]
    fn qualified_values_round_trip_through_serde() {
        let criterion = ScientificMilestoneCriterion::new(
            "milestone:dataset",
            ScientificMilestoneKind::DatasetDeposited,
        )
        .unwrap();
        let json = serde_json::to_string(&criterion).unwrap();
        let decoded: ScientificMilestoneCriterion = serde_json::from_str(&json).unwrap();
        assert_eq!(criterion, decoded);
    }
}
