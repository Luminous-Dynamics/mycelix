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

    /// Convert a condition observation into an AC-017 substrate account.
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
