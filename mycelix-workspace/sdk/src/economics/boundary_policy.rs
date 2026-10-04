// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Boundary Governance
//!
//! Reference model for governing changes to substrate safety boundaries.
//!
//! A substrate ledger is not enough if the entity being evaluated can silently
//! redefine the boundary until the observed state appears healthy. This module
//! therefore treats boundary definitions as versioned governance objects.
//!
//! Core invariant:
//!
//! boundary relaxation != ordinary configuration update
//!
//! Tightening a boundary can happen immediately through the ordinary local
//! authority. Relaxation requires explicit independent authority, evidence,
//! and a cooling period supplied by the governing policy.

use super::substrate::{BoundaryDirection, SubstrateBoundary, SubstrateDimension};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Reason for a boundary revision.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BoundaryChangeKind {
    /// First boundary definition for a scope/dimension.
    Initial,
    /// Makes the safety condition more protective.
    Tighten,
    /// Makes the safety condition less protective.
    Relax,
    /// Revises the boundary because better measurement/evidence changed the
    /// defensible value; still subject to anti-capture rules if it relaxes.
    EvidenceCorrection,
}

/// Versioned substrate boundary definition.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BoundaryRevision {
    /// Unique revision identifier.
    pub id: String,
    /// Resource/institutional scope governed by this boundary.
    pub scope_ref: String,
    /// Actor or organization whose activity is governed.
    pub subject_ref: String,
    /// Substrate dimension governed by this boundary.
    pub dimension: SubstrateDimension,
    /// Native unit for the boundary.
    pub unit: String,
    /// Minimum/maximum orientation.
    pub direction: BoundaryDirection,
    /// Boundary value in native integer units.
    pub boundary: i128,
    /// Early warning distance.
    pub warning_buffer: u128,
    /// Whether crossing the boundary is policy-blocking.
    pub hard: bool,
    /// Revision kind.
    pub change_kind: BoundaryChangeKind,
    /// Previous revision ID, absent for the initial definition.
    pub previous_revision_id: Option<String>,
    /// Timestamp when change was proposed.
    pub proposed_at: u64,
    /// Timestamp when change becomes effective.
    pub effective_at: u64,
    /// Authorities supporting the revision.
    pub authority_refs: Vec<String>,
    /// Authorities independent of the subject.
    pub independent_authority_refs: Vec<String>,
    /// Evidence supporting the revision.
    pub evidence_refs: Vec<String>,
    /// Human-readable rationale.
    pub rationale: String,
}

impl BoundaryRevision {
    /// Convert to the substrate boundary primitive.
    pub fn boundary(&self) -> SubstrateBoundary {
        SubstrateBoundary {
            direction: self.direction,
            boundary: self.boundary,
            warning_buffer: self.warning_buffer,
            hard: self.hard,
        }
    }

    fn validate_identity(&self) -> Result<(), String> {
        for (name, value) in [
            ("revision id", &self.id),
            ("scope", &self.scope_ref),
            ("subject", &self.subject_ref),
            ("unit", &self.unit),
            ("rationale", &self.rationale),
        ] {
            if value.trim().is_empty() {
                return Err(format!("Boundary {name} cannot be empty"));
            }
        }

        if self.authority_refs.is_empty() {
            return Err("Boundary revision requires authority references".into());
        }
        if self.evidence_refs.is_empty() {
            return Err("Boundary revision requires evidence references".into());
        }
        if self.proposed_at > self.effective_at {
            return Err("Boundary effective time cannot precede proposal time".into());
        }

        validate_unique_nonempty("authority", &self.authority_refs)?;
        validate_unique_nonempty("independent authority", &self.independent_authority_refs)?;
        validate_unique_nonempty("evidence", &self.evidence_refs)?;

        let authorities: BTreeSet<_> = self.authority_refs.iter().collect();
        for independent in &self.independent_authority_refs {
            if !authorities.contains(independent) {
                return Err(format!(
                    "Independent authority '{}' is not present in authority_refs",
                    independent
                ));
            }
        }

        Ok(())
    }
}

/// Local policy controlling how substrate boundaries may evolve.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BoundaryGovernancePolicy {
    /// Minimum delay before a relaxation can become effective.
    pub minimum_relaxation_cooldown_us: u64,
    /// Require at least one authority independent of the subject for relaxations.
    pub require_independent_authority: bool,
}

impl Default for BoundaryGovernancePolicy {
    fn default() -> Self {
        Self {
            minimum_relaxation_cooldown_us: 7 * 24 * 3600 * 1_000_000,
            require_independent_authority: true,
        }
    }
}

/// Errors from boundary governance validation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum BoundaryGovernanceError {
    InvalidRevision(String),
    MissingPredecessor,
    PredecessorIdMismatch,
    ScopeMismatch,
    SubjectMismatch,
    DimensionMismatch,
    UnitMismatch,
    DirectionMismatch,
    RetroactiveChange,
    DuplicateRevision,
    DuplicateEffectiveTime,
    ChangeKindMismatch,
    RelaxationNeedsIndependentAuthority,
    RelaxationCoolingPeriod,
}

/// Contract for boundary revisions.
#[derive(Debug, Default, Clone, Copy)]
pub struct BoundaryGovernanceContract;

impl BoundaryGovernanceContract {
    /// Validate an initial boundary revision.
    pub fn validate_initial(
        revision: &BoundaryRevision,
    ) -> Result<(), BoundaryGovernanceError> {
        revision
            .validate_identity()
            .map_err(BoundaryGovernanceError::InvalidRevision)?;

        if revision.change_kind != BoundaryChangeKind::Initial {
            return Err(BoundaryGovernanceError::ChangeKindMismatch);
        }
        if revision.previous_revision_id.is_some() {
            return Err(BoundaryGovernanceError::PredecessorIdMismatch);
        }

        Ok(())
    }

    /// Validate a successor revision against its predecessor.
    pub fn validate_successor(
        predecessor: &BoundaryRevision,
        successor: &BoundaryRevision,
        policy: &BoundaryGovernancePolicy,
    ) -> Result<(), BoundaryGovernanceError> {
        successor
            .validate_identity()
            .map_err(BoundaryGovernanceError::InvalidRevision)?;

        if successor.previous_revision_id.as_deref() != Some(predecessor.id.as_str()) {
            return Err(BoundaryGovernanceError::PredecessorIdMismatch);
        }
        if successor.scope_ref != predecessor.scope_ref {
            return Err(BoundaryGovernanceError::ScopeMismatch);
        }
        if successor.subject_ref != predecessor.subject_ref {
            return Err(BoundaryGovernanceError::SubjectMismatch);
        }
        if successor.dimension != predecessor.dimension {
            return Err(BoundaryGovernanceError::DimensionMismatch);
        }
        if successor.unit != predecessor.unit {
            return Err(BoundaryGovernanceError::UnitMismatch);
        }
        if successor.direction != predecessor.direction {
            return Err(BoundaryGovernanceError::DirectionMismatch);
        }
        if successor.id == predecessor.id || successor.proposed_at < predecessor.effective_at {
            return Err(BoundaryGovernanceError::RetroactiveChange);
        }

        let relaxing = is_relaxation(predecessor, successor);
        let tightening = is_tightening(predecessor, successor);

        match successor.change_kind {
            BoundaryChangeKind::Initial => {
                return Err(BoundaryGovernanceError::ChangeKindMismatch);
            }
            BoundaryChangeKind::Tighten if !tightening || relaxing => {
                return Err(BoundaryGovernanceError::ChangeKindMismatch);
            }
            BoundaryChangeKind::Relax if !relaxing => {
                return Err(BoundaryGovernanceError::ChangeKindMismatch);
            }
            BoundaryChangeKind::EvidenceCorrection => {}
            _ => {}
        }

        if relaxing {
            if policy.require_independent_authority {
                let independent = successor
                    .independent_authority_refs
                    .iter()
                    .any(|authority| authority != &successor.subject_ref);
                if !independent {
                    return Err(BoundaryGovernanceError::RelaxationNeedsIndependentAuthority);
                }
            }

            let earliest = successor
                .proposed_at
                .checked_add(policy.minimum_relaxation_cooldown_us)
                .ok_or(BoundaryGovernanceError::RelaxationCoolingPeriod)?;
            if successor.effective_at < earliest {
                return Err(BoundaryGovernanceError::RelaxationCoolingPeriod);
            }
        }

        Ok(())
    }
}

/// Determine whether the new boundary is less protective.
fn is_relaxation(previous: &BoundaryRevision, next: &BoundaryRevision) -> bool {
    let value_relaxes = match previous.direction {
        BoundaryDirection::Minimum => next.boundary < previous.boundary,
        BoundaryDirection::Maximum => next.boundary > previous.boundary,
    };

    let hard_relaxes = previous.hard && !next.hard;
    let warning_relaxes = next.warning_buffer < previous.warning_buffer;

    value_relaxes || hard_relaxes || warning_relaxes
}

/// Determine whether the new boundary is more protective.
fn is_tightening(previous: &BoundaryRevision, next: &BoundaryRevision) -> bool {
    let value_tightens = match previous.direction {
        BoundaryDirection::Minimum => next.boundary > previous.boundary,
        BoundaryDirection::Maximum => next.boundary < previous.boundary,
    };

    let hard_tightens = !previous.hard && next.hard;
    let warning_tightens = next.warning_buffer > previous.warning_buffer;

    value_tightens || hard_tightens || warning_tightens
}

fn validate_unique_nonempty(kind: &str, values: &[String]) -> Result<(), String> {
    let mut seen = BTreeSet::new();
    for value in values {
        if value.trim().is_empty() {
            return Err(format!("{kind} reference cannot be empty"));
        }
        if !seen.insert(value) {
            return Err(format!("Duplicate {kind} reference: {value}"));
        }
    }
    Ok(())
}

/// In-memory reference ledger for deterministic revision chaining.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BoundaryLedger {
    revisions: BTreeMap<String, BoundaryRevision>,
}

impl BoundaryLedger {
    /// Create an empty ledger.
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a boundary revision.
    pub fn insert(
        &mut self,
        revision: BoundaryRevision,
        policy: &BoundaryGovernancePolicy,
    ) -> Result<(), BoundaryGovernanceError> {
        if self.revisions.contains_key(&revision.id) {
            return Err(BoundaryGovernanceError::DuplicateRevision);
        }

        if self.revisions.values().any(|existing| {
            existing.scope_ref == revision.scope_ref
                && existing.dimension == revision.dimension
                && existing.effective_at == revision.effective_at
        }) {
            return Err(BoundaryGovernanceError::DuplicateEffectiveTime);
        }

        match revision.previous_revision_id.as_deref() {
            None => BoundaryGovernanceContract::validate_initial(&revision)?,
            Some(previous_id) => {
                let previous = self
                    .revisions
                    .get(previous_id)
                    .ok_or(BoundaryGovernanceError::MissingPredecessor)?;
                BoundaryGovernanceContract::validate_successor(previous, &revision, policy)?;

            }
        }

        self.revisions.insert(revision.id.clone(), revision);
        Ok(())
    }

    /// Get the revision that is currently effective at a timestamp.
    pub fn current(
        &self,
        scope_ref: &str,
        dimension: SubstrateDimension,
        at: u64,
    ) -> Option<&BoundaryRevision> {
        self.revisions
            .values()
            .filter(|revision| {
                revision.scope_ref == scope_ref
                    && revision.dimension == dimension
                    && revision.effective_at <= at
            })
            .max_by_key(|revision| (revision.effective_at, revision.id.as_str()))
    }

    /// Return all stored revisions in canonical ID order.
    pub fn revisions(&self) -> impl Iterator<Item = &BoundaryRevision> {
        self.revisions.values()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn initial() -> BoundaryRevision {
        BoundaryRevision {
            id: "b-1".into(),
            scope_ref: "river:1".into(),
            subject_ref: "organization:a".into(),
            dimension: SubstrateDimension::Ecological,
            unit: "m3".into(),
            direction: BoundaryDirection::Minimum,
            boundary: 700,
            warning_buffer: 100,
            hard: true,
            change_kind: BoundaryChangeKind::Initial,
            previous_revision_id: None,
            proposed_at: 1_000,
            effective_at: 1_000,
            authority_refs: vec!["council:a".into()],
            independent_authority_refs: vec!["council:a".into()],
            evidence_refs: vec!["evidence:initial".into()],
            rationale: "Initial measured safety floor".into(),
        }
    }

    fn relaxed(previous: &BoundaryRevision) -> BoundaryRevision {
        BoundaryRevision {
            id: "b-2".into(),
            scope_ref: previous.scope_ref.clone(),
            subject_ref: previous.subject_ref.clone(),
            dimension: previous.dimension,
            unit: previous.unit.clone(),
            direction: previous.direction,
            boundary: 600,
            warning_buffer: 100,
            hard: true,
            change_kind: BoundaryChangeKind::Relax,
            previous_revision_id: Some(previous.id.clone()),
            proposed_at: 2_000,
            effective_at: 2_000 + 7 * 24 * 3600 * 1_000_000,
            authority_refs: vec!["council:a".into(), "ombuds:1".into()],
            independent_authority_refs: vec!["ombuds:1".into()],
            evidence_refs: vec!["evidence:new".into()],
            rationale: "New evidence supports a lower defensible floor".into(),
        }
    }

    #[test]
    fn initial_boundary_is_valid() {
        BoundaryGovernanceContract::validate_initial(&initial()).unwrap();
    }

    #[test]
    fn relaxation_requires_independent_authority() {
        let previous = initial();
        let mut next = relaxed(&previous);
        next.authority_refs = vec!["council:a".into()];
        next.independent_authority_refs = vec!["council:a".into()];

        let result = BoundaryGovernanceContract::validate_successor(
            &previous,
            &next,
            &BoundaryGovernancePolicy::default(),
        );

        assert_eq!(
            result,
            Err(BoundaryGovernanceError::RelaxationNeedsIndependentAuthority)
        );
    }

    #[test]
    fn relaxation_requires_cooling_period() {
        let previous = initial();
        let mut next = relaxed(&previous);
        next.effective_at = next.proposed_at;

        let result = BoundaryGovernanceContract::validate_successor(
            &previous,
            &next,
            &BoundaryGovernancePolicy::default(),
        );

        assert_eq!(
            result,
            Err(BoundaryGovernanceError::RelaxationCoolingPeriod)
        );
    }

    #[test]
    fn tightening_does_not_require_relaxation_delay() {
        let previous = initial();
        let next = BoundaryRevision {
            id: "b-2-tighten".into(),
            boundary: 800,
            change_kind: BoundaryChangeKind::Tighten,
            previous_revision_id: Some(previous.id.clone()),
            proposed_at: 2_000,
            effective_at: 2_000,
            ..previous.clone()
        };

        BoundaryGovernanceContract::validate_successor(
            &previous,
            &next,
            &BoundaryGovernancePolicy::default(),
        )
        .unwrap();
    }

    #[test]
    fn retroactive_revision_is_rejected() {
        let previous = initial();
        let mut next = relaxed(&previous);
        next.proposed_at = 900;
        next.effective_at = 900;

        assert_eq!(
            BoundaryGovernanceContract::validate_successor(
                &previous,
                &next,
                &BoundaryGovernancePolicy::default()
            ),
            Err(BoundaryGovernanceError::RetroactiveChange)
        );
    }

    #[test]
    fn ledger_current_is_deterministic() {
        let policy = BoundaryGovernancePolicy::default();
        let first = initial();
        let second = relaxed(&first);

        let mut left = BoundaryLedger::new();
        left.insert(first.clone(), &policy).unwrap();
        left.insert(second.clone(), &policy).unwrap();

        let mut right = BoundaryLedger::new();
        right.insert(first.clone(), &policy).unwrap();
        right.insert(second.clone(), &policy).unwrap();

        assert_eq!(
            left.current("river:1", SubstrateDimension::Ecological, 1_000)
                .unwrap()
                .id,
            right
                .current("river:1", SubstrateDimension::Ecological, 1_000)
                .unwrap()
                .id
        );
        assert_eq!(
            left.current(
                "river:1",
                SubstrateDimension::Ecological,
                2_000 + 7 * 24 * 3600 * 1_000_000
            )
            .unwrap()
            .id,
            right
                .current(
                    "river:1",
                    SubstrateDimension::Ecological,
                    2_000 + 7 * 24 * 3600 * 1_000_000
                )
                .unwrap()
                .id
        );
    }

    #[test]
    fn duplicate_initial_effective_time_is_rejected() {
        let policy = BoundaryGovernancePolicy::default();
        let mut ledger = BoundaryLedger::new();
        ledger.insert(initial(), &policy).unwrap();

        let duplicate_scope = BoundaryRevision {
            id: "b-duplicate".into(),
            ..initial()
        };

        assert_eq!(
            ledger.insert(duplicate_scope, &policy),
            Err(BoundaryGovernanceError::DuplicateEffectiveTime)
        );
    }
    }
}
