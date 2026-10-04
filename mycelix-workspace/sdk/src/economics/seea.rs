// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # SEEA Ecosystem Accounting Interoperability
//!
//! Typed evidence boundary for ingesting SEEA-shaped ecosystem observations
//! without turning an environmental accounting standard into a Mycelix score.
//!
//! SEEA EA defines ecosystem extent, ecosystem condition, ecosystem services,
//! and ecosystem asset accounts. This module keeps those account concepts
//! explicit and preserves native units and source provenance.
//!
//! This is an interoperability/reference layer, not a replacement for SEEA.

use super::substrate::{SubstrateAccount, SubstrateBoundary, SubstrateDimension};
use serde::{Deserialize, Serialize};

/// SEEA ecosystem account family represented by an observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SeeaAccountType {
    /// Ecosystem extent account.
    Extent,
    /// Ecosystem condition account.
    Condition,
    /// Physical ecosystem-service flow account.
    ServicesPhysical,
    /// Monetary ecosystem-service flow account.
    ServicesMonetary,
    /// Ecosystem monetary-asset account.
    EcosystemAsset,
}

/// Observation direction relative to the account's reference period.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SeeaChangeKind {
    /// Observation establishes or refreshes a stock/reference state.
    Stock,
    /// Observation reports an increase.
    Increase,
    /// Observation reports a decrease.
    Decrease,
}

/// Result of applying an explicit freshness policy to an observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SeeaFreshness {
    /// Observation falls within the permitted age window.
    Current,
    /// Observation is older than the permitted age window.
    Stale,
}

/// Error returned when a SEEA observation cannot be treated as current evidence.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum SeeaFreshnessError {
    /// Decision timestamp precedes the end of the observation period.
    ObservationFromFuture,
    /// Observation is outside the configured freshness window.
    StaleObservation,
    /// A publisher timestamp is later than the decision timestamp.
    SourceTimestampFromFuture,
}

/// Explicit temporal policy for using a SEEA observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SeeaFreshnessPolicy {
    /// Decision time in the same units used by the observation.
    pub as_of: u64,
    /// Maximum permitted age measured from period end.
    pub max_age: u64,
}

impl SeeaFreshnessPolicy {
    /// Classify an observation without changing its underlying value.
    pub fn evaluate(
        &self,
        observation: &SeeaObservation,
    ) -> Result<SeeaFreshness, SeeaFreshnessError> {
        if observation.period_end > self.as_of {
            return Err(SeeaFreshnessError::ObservationFromFuture);
        }

        if observation
            .provenance
            .source_timestamp
            .is_some_and(|timestamp| timestamp > self.as_of)
        {
            return Err(SeeaFreshnessError::SourceTimestampFromFuture);
        }

        let age = self.as_of.saturating_sub(observation.period_end);
        if age > self.max_age {
            Ok(SeeaFreshness::Stale)
        } else {
            Ok(SeeaFreshness::Current)
        }
    }

    /// Require an observation to be current.
    pub fn require_current(
        &self,
        observation: &SeeaObservation,
    ) -> Result<(), SeeaFreshnessError> {
        match self.evaluate(observation)? {
            SeeaFreshness::Current => Ok(()),
            SeeaFreshness::Stale => Err(SeeaFreshnessError::StaleObservation),
        }
    }
}

/// Source provenance carried with an imported observation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SeeaProvenance {
    /// Source organization or dataset identifier.
    pub source_ref: String,
    /// Immutable content/version reference where available.
    pub content_hash: Option<String>,
    /// Version of the source schema or publication.
    pub schema_version: String,
    /// Optional retrieval/publication timestamp.
    pub source_timestamp: Option<u64>,
}

impl SeeaProvenance {
    /// Validate the minimum provenance envelope.
    pub fn validate(&self) -> Result<(), String> {
        if self.source_ref.trim().is_empty() {
            return Err("SEEA source reference cannot be empty".into());
        }
        if self.schema_version.trim().is_empty() {
            return Err("SEEA schema version cannot be empty".into());
        }
        if self
            .content_hash
            .as_ref()
            .is_some_and(|hash| hash.trim().is_empty())
        {
            return Err("SEEA content hash cannot be empty when present".into());
        }
        Ok(())
    }
}

/// SEEA-shaped ecosystem accounting observation.
///
/// The value is always represented in its published native unit. No automatic
/// currency conversion or ecological price assignment occurs here.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SeeaObservation {
    /// Stable observation identifier.
    pub id: String,
    /// Account family.
    pub account_type: SeeaAccountType,
    /// Ecosystem accounting area, e.g. catchment or municipality.
    pub accounting_area_ref: String,
    /// Ecosystem asset/type identifier.
    pub ecosystem_type_ref: String,
    /// Economic or institutional unit using/supplying the account, if known.
    pub economic_unit_ref: Option<String>,
    /// Native measurement unit.
    pub unit: String,
    /// Observation/reference value in native integer units.
    pub value: i128,
    /// Optional reference level used by the publisher.
    pub reference_value: Option<i128>,
    /// Stock/increase/decrease semantic.
    pub change_kind: SeeaChangeKind,
    /// Start of accounting period.
    pub period_start: u64,
    /// End of accounting period.
    pub period_end: u64,
    /// Source provenance.
    pub provenance: SeeaProvenance,
}

impl SeeaObservation {
    /// Validate structural integrity without judging substantive correctness.
    pub fn validate(&self) -> Result<(), String> {
        if self.id.trim().is_empty() {
            return Err("SEEA observation ID cannot be empty".into());
        }
        if self.accounting_area_ref.trim().is_empty() {
            return Err("SEEA accounting area cannot be empty".into());
        }
        if self.ecosystem_type_ref.trim().is_empty() {
            return Err("SEEA ecosystem type cannot be empty".into());
        }
        if self.unit.trim().is_empty() {
            return Err("SEEA unit cannot be empty".into());
        }
        if self.period_start >= self.period_end {
            return Err("SEEA period must have positive duration".into());
        }
        self.provenance.validate()
    }

    /// Convert a condition observation into an AC-017 substrate account after
    /// explicit freshness qualification.
    ///
    /// Only condition accounts map directly to the ecological substrate
    /// dimension. Extent/services/asset accounts remain typed observations
    /// until local policy specifies how they should constrain actions.
    pub fn to_ecological_substrate(
        &self,
        boundary: SubstrateBoundary,
        baseline: i128,
        updated_at: u64,
    ) -> Result<SubstrateAccount, String> {
        self.validate()?;

        if self.account_type != SeeaAccountType::Condition {
            return Err(
                "Only SEEA condition observations map directly to ecological substrate state"
                    .into(),
            );
        }

        Ok(SubstrateAccount::new(
            SubstrateDimension::Ecological,
            self.unit.clone(),
            baseline,
            self.value,
            boundary,
            updated_at,
        ))
    }


    /// Convert to ecological substrate only when the observation is explicitly
    /// accepted by the caller's freshness policy.
    pub fn to_ecological_substrate_if_current(
        &self,
        boundary: SubstrateBoundary,
        baseline: i128,
        updated_at: u64,
        freshness: &SeeaFreshnessPolicy,
    ) -> Result<SubstrateAccount, String> {
        freshness
            .require_current(self)
            .map_err(|error| format!("SEEA freshness qualification failed: {error:?}"))?;

        self.to_ecological_substrate(boundary, baseline, updated_at)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn condition_observation(value: i128) -> SeeaObservation {
        SeeaObservation {
            id: "seea:condition:river-1:2026".into(),
            account_type: SeeaAccountType::Condition,
            accounting_area_ref: "catchment:river-1".into(),
            ecosystem_type_ref: "inland-water".into(),
            economic_unit_ref: None,
            unit: "condition-index".into(),
            value,
            reference_value: Some(900),
            change_kind: SeeaChangeKind::Stock,
            period_start: 1_700,
            period_end: 1_800,
            provenance: SeeaProvenance {
                source_ref: "stats:ecosystem-account".into(),
                content_hash: Some("sha256:abc".into()),
                schema_version: "SEEA-EA:2021".into(),
                source_timestamp: Some(1_801),
            },
        }
    }

    #[test]
    fn condition_observation_maps_to_ecological_substrate() {
        let observation = condition_observation(760);
        let account = observation
            .to_ecological_substrate(
                SubstrateBoundary::minimum(700, 50, true),
                900,
                1_801,
            )
            .unwrap();

        assert_eq!(account.dimension, SubstrateDimension::Ecological);
        assert_eq!(account.current, 760);
        assert_eq!(account.unit, "condition-index");
    }

    #[test]
    fn non_condition_accounts_do_not_gain_hidden_policy_meaning() {
        let mut observation = condition_observation(760);
        observation.account_type = SeeaAccountType::ServicesPhysical;

        let result = observation.to_ecological_substrate(
            SubstrateBoundary::minimum(700, 50, true),
            900,
            1_801,
        );

        assert!(result.is_err());
    }

    #[test]
    fn stale_observation_does_not_become_current_without_policy_override() {
        let observation = condition_observation(760);
        let policy = SeeaFreshnessPolicy {
            as_of: 2_000,
            max_age: 100,
        };

        assert_eq!(policy.evaluate(&observation), Ok(SeeaFreshness::Stale));
        assert_eq!(
            observation
                .to_ecological_substrate_if_current(
                    SubstrateBoundary::minimum(700, 50, true),
                    900,
                    1_801,
                    &policy
                )
                .is_err(),
            true
        );
    }

    #[test]
    fn future_source_timestamp_is_rejected() {
        let observation = condition_observation(760);
        let policy = SeeaFreshnessPolicy {
            as_of: 1_800,
            max_age: 100,
        };

        assert_eq!(
            policy.evaluate(&observation),
            Err(SeeaFreshnessError::SourceTimestampFromFuture)
        );
    }

    #[test]
    fn current_observation_can_be_projected_after_freshness_check() {
        let observation = condition_observation(760);
        let policy = SeeaFreshnessPolicy {
            as_of: 1_820,
            max_age: 50,
        };

        assert!(
            observation
                .to_ecological_substrate_if_current(
                    SubstrateBoundary::minimum(700, 50, true),
                    900,
                    1_801,
                    &policy
                )
                .is_ok()
        );
    }

    #[test]
    fn invalid_provenance_fails_closed() {
        let mut observation = condition_observation(760);
        observation.provenance.source_ref.clear();

        assert!(observation.validate().is_err());
    }

    #[test]
    fn signed_values_are_preserved_exactly() {
        let observation = condition_observation(-7);
        assert_eq!(observation.value, -7);
    }
}
