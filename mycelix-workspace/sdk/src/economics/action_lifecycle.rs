// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Economic Action Lifecycle
//!
//! Append-only lifecycle binding for a scoped economic action.
//!
//! AC-030 makes action scope explicit. AC-035 ensures that the same action
//! identity and scope identity remain attached as an action moves from planning
//! through execution and completion. Legitimate scope changes are represented
//! as explicit amendments instead of silently replacing the active scope.
//!
//! This is a reference model. It does not prescribe procurement law or local
//! approval policy.

use super::integrity_gate::EconomicActionScope;
use serde::{Deserialize, Serialize};

/// Lifecycle stage of an economic action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EconomicActionStage {
    /// Action is being defined or budgeted.
    Planning,
    /// Suppliers or execution options are being evaluated.
    Tendering,
    /// A provider or execution path has been selected.
    Awarded,
    /// A binding agreement exists.
    Contracted,
    /// The action is being carried out.
    Implementation,
    /// The action completed normally.
    Completed,
    /// The action was terminated before normal completion.
    Terminated,
}

/// Why a lifecycle revision exists.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EconomicActionChangeKind {
    /// First publication of the action.
    Initial,
    /// Non-scope lifecycle update.
    Update,
    /// Explicit amendment to the active action scope.
    ScopeAmendment,
    /// Normal completion.
    Completion,
    /// Explicit termination.
    Termination,
}

/// Immutable lifecycle revision for an economic action.
///
/// A revision never edits a previous revision. The predecessor reference forms
/// a linear history and the action reference forms the stable identity joining
/// all stages.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicActionRevision {
    /// Immutable revision identifier.
    pub revision_id: String,
    /// Stable action identifier.
    pub action_ref: String,
    /// Scope identifier active for this revision.
    pub scope_id: String,
    /// SHA-256 fingerprint of the complete scope contents.
    pub scope_fingerprint: String,
    /// Lifecycle stage represented by this revision.
    pub stage: EconomicActionStage,
    /// Semantic reason for the revision.
    pub kind: EconomicActionChangeKind,
    /// Immediately preceding revision.
    pub predecessor_revision_id: Option<String>,
    /// Scope identifier active before a scope amendment.
    pub predecessor_scope_id: Option<String>,
    /// Authority responsible for this revision.
    pub authority_ref: String,
    /// Evidence supporting the revision.
    pub evidence_refs: Vec<String>,
    /// Monotonic lifecycle timestamp.
    pub recorded_at: u64,
}

impl EconomicActionRevision {
    /// Validate immutable revision structure.
    pub fn validate(&self) -> Result<(), String> {
        if self.revision_id.trim().is_empty() {
            return Err("Lifecycle revision ID cannot be empty".into());
        }
        if self.action_ref.trim().is_empty() {
            return Err("Lifecycle action reference cannot be empty".into());
        }
        if self.scope_id.trim().is_empty() {
            return Err("Lifecycle scope ID cannot be empty".into());
        }
        if self.scope_fingerprint.trim().is_empty() {
            return Err("Lifecycle scope fingerprint cannot be empty".into());
        }
        if self.authority_ref.trim().is_empty() {
            return Err("Lifecycle authority reference cannot be empty".into());
        }
        if self
            .evidence_refs
            .iter()
            .any(|reference| reference.trim().is_empty())
        {
            return Err("Lifecycle evidence references cannot be empty".into());
        }

        match self.kind {
            EconomicActionChangeKind::Initial => {
                if self.stage != EconomicActionStage::Planning {
                    return Err("Initial lifecycle revision must be Planning".into());
                }
                if self.predecessor_revision_id.is_some()
                    || self.predecessor_scope_id.is_some()
                {
                    return Err("Initial lifecycle revision cannot have predecessors".into());
                }
            }
            EconomicActionChangeKind::ScopeAmendment => {
                if self.predecessor_scope_id.is_none() {
                    return Err("Scope amendment requires the predecessor scope".into());
                }
                if self.predecessor_scope_id.as_deref() == Some(self.scope_id.as_str()) {
                    return Err("Scope amendment must change the scope ID".into());
                }
            }
            EconomicActionChangeKind::Completion => {
                if self.stage != EconomicActionStage::Completed {
                    return Err("Completion revision must use Completed stage".into());
                }
            }
            EconomicActionChangeKind::Termination => {
                if self.stage != EconomicActionStage::Terminated {
                    return Err("Termination revision must use Terminated stage".into());
                }
            }
            EconomicActionChangeKind::Update => {
                if self.predecessor_scope_id.is_some() {
                    return Err("Ordinary update cannot carry a predecessor scope".into());
                }
            }
        }

        Ok(())
    }
}

/// Append-only lifecycle ledger for one economic action.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicActionLifecycle {
    action_ref: String,
    active_scope_id: String,
    current_revision_id: String,
    active_scope_fingerprint: String,
    current_stage: EconomicActionStage,
    revisions: Vec<EconomicActionRevision>,
}

impl EconomicActionLifecycle {
    /// Start a lifecycle with an explicit AC-030 scope.
    pub fn start(
        scope: &EconomicActionScope,
        revision_id: impl Into<String>,
        authority_ref: impl Into<String>,
        evidence_refs: Vec<String>,
        recorded_at: u64,
    ) -> Result<Self, String> {
        scope.validate()?;
        let scope_fingerprint = scope.fingerprint()?;

        let revision = EconomicActionRevision {
            revision_id: revision_id.into(),
            action_ref: scope.action_ref.clone(),
            scope_id: scope.scope_id.clone(),
            scope_fingerprint: scope_fingerprint.clone(),
            stage: EconomicActionStage::Planning,
            kind: EconomicActionChangeKind::Initial,
            predecessor_revision_id: None,
            predecessor_scope_id: None,
            authority_ref: authority_ref.into(),
            evidence_refs,
            recorded_at,
        };
        revision.validate()?;

        Ok(Self {
            action_ref: scope.action_ref.clone(),
            active_scope_id: scope.scope_id.clone(),
            current_revision_id: revision.revision_id.clone(),
            active_scope_fingerprint: scope_fingerprint,
            current_stage: EconomicActionStage::Planning,
            revisions: vec![revision],
        })
    }

    /// Read the stable action identity.
    pub fn action_ref(&self) -> &str {
        &self.action_ref
    }

    /// Read the active scope identity.
    pub fn active_scope_id(&self) -> &str {
        &self.active_scope_id
    }

    /// Validate the lifecycle's internal consistency.
    pub fn validate(&self) -> Result<(), String> {
        if self.action_ref.trim().is_empty() {
            return Err("Lifecycle action reference cannot be empty".into());
        }
        if self.active_scope_id.trim().is_empty() {
            return Err("Lifecycle active scope ID cannot be empty".into());
        }
        if self.active_scope_fingerprint.trim().is_empty() {
            return Err("Lifecycle active scope fingerprint cannot be empty".into());
        }
        if self.current_revision_id.trim().is_empty() {
            return Err("Lifecycle current revision ID cannot be empty".into());
        }
        if self.revisions.is_empty() {
            return Err("Lifecycle must contain at least one revision".into());
        }

        let first = &self.revisions[0];
        first.validate()?;
        if first.action_ref != self.action_ref {
            return Err("Lifecycle action reference does not match first revision".into());
        }
        if first.kind != EconomicActionChangeKind::Initial {
            return Err("Lifecycle first revision must be Initial".into());
        }

        for window in self.revisions.windows(2) {
            let previous = &window[0];
            let current = &window[1];
            if current.predecessor_revision_id.as_deref() != Some(previous.revision_id.as_str()) {
                return Err("Lifecycle revision chain is not linear".into());
            }
            if current.recorded_at < previous.recorded_at {
                return Err("Lifecycle timestamps are not monotonic".into());
            }
            if current.action_ref != self.action_ref {
                return Err("Lifecycle action reference changed in history".into());
            }
            current.validate()?;
        }

        let last = self.revisions.last().unwrap();
        if self.current_revision_id != last.revision_id {
            return Err("Lifecycle current revision does not match history".into());
        }
        if self.current_stage != last.stage {
            return Err("Lifecycle current stage does not match history".into());
        }
        if self.active_scope_id != last.scope_id {
            return Err("Lifecycle active scope does not match history".into());
        }
        if self.active_scope_fingerprint != last.scope_fingerprint {
            return Err("Lifecycle active scope fingerprint does not match history".into());
        }
        Ok(())
    }

    /// Read the current revision when the lifecycle is structurally valid.
    pub fn current_revision(&self) -> Result<&EconomicActionRevision, String> {
        self.revisions
            .last()
            .ok_or_else(|| "Lifecycle has no revisions".to_string())
    }

    /// Read the current stage.
    pub fn current_stage(&self) -> EconomicActionStage {
        self.current_stage
    }

    /// Read the immutable history.
    pub fn revisions(&self) -> &[EconomicActionRevision] {
        &self.revisions
    }

    /// Record a normal lifecycle update or terminal transition.
    ///
    /// A normal update must retain the active scope. Any scope change must use
    /// amend_scope so the predecessor scope is explicit.
    pub fn record(
        &mut self,
        revision_id: impl Into<String>,
        stage: EconomicActionStage,
        kind: EconomicActionChangeKind,
        scope: &EconomicActionScope,
        authority_ref: impl Into<String>,
        evidence_refs: Vec<String>,
        recorded_at: u64,
    ) -> Result<(), String> {
        scope.validate()?;
        if scope.action_ref != self.action_ref {
            return Err("Lifecycle update must retain the action reference".into());
        }
        if scope.scope_id != self.active_scope_id {
            return Err(
                "Lifecycle scope cannot change during a normal update; use amend_scope instead"
                    .into(),
            );
        }
        let scope_fingerprint = scope.fingerprint()?;
        if scope_fingerprint != self.active_scope_fingerprint {
            return Err(
                "Lifecycle scope contents changed without an explicit scope amendment".into(),
            );
        }

        self.record_inner(EconomicActionRevision {
            revision_id: revision_id.into(),
            action_ref: self.action_ref.clone(),
            scope_id: scope.scope_id.clone(),
            scope_fingerprint,
            stage,
            kind,
            predecessor_revision_id: Some(self.current_revision_id.clone()),
            predecessor_scope_id: None,
            authority_ref: authority_ref.into(),
            evidence_refs,
            recorded_at,
        })
    }

    /// Amend the active scope without changing the stable action identity.
    pub fn amend_scope(
        &mut self,
        new_scope: &EconomicActionScope,
        revision_id: impl Into<String>,
        authority_ref: impl Into<String>,
        evidence_refs: Vec<String>,
        recorded_at: u64,
    ) -> Result<(), String> {
        new_scope.validate()?;
        let new_scope_fingerprint = new_scope.fingerprint()?;
        if new_scope.action_ref != self.action_ref {
            return Err("Scope amendment must retain the lifecycle action reference".into());
        }
        if new_scope.scope_id == self.active_scope_id {
            return Err("Scope amendment must create a new scope ID".into());
        }
        if matches!(
            self.current_stage,
            EconomicActionStage::Completed | EconomicActionStage::Terminated
        ) {
            return Err("Terminal actions cannot be scope-amended".into());
        }

        self.record_inner(EconomicActionRevision {
            revision_id: revision_id.into(),
            action_ref: self.action_ref.clone(),
            scope_id: new_scope.scope_id.clone(),
            scope_fingerprint: new_scope_fingerprint,
            stage: self.current_stage,
            kind: EconomicActionChangeKind::ScopeAmendment,
            predecessor_revision_id: Some(self.current_revision_id.clone()),
            predecessor_scope_id: Some(self.active_scope_id.clone()),
            authority_ref: authority_ref.into(),
            evidence_refs,
            recorded_at,
        })
    }

    fn record_inner(&mut self, revision: EconomicActionRevision) -> Result<(), String> {
        revision.validate()?;

        if revision.action_ref != self.action_ref {
            return Err("Lifecycle revision action reference cannot change".into());
        }
        if revision.revision_id == self.current_revision_id
            || self
                .revisions
                .iter()
                .any(|existing| existing.revision_id == revision.revision_id)
        {
            return Err(format!(
                "Duplicate lifecycle revision ID: {}",
                revision.revision_id
            ));
        }
        if revision.predecessor_revision_id.as_deref() != Some(self.current_revision_id.as_str()) {
            return Err("Lifecycle revision predecessor does not match current revision".into());
        }
        if revision.recorded_at < self.current_revision()?.recorded_at {
            return Err("Lifecycle timestamp cannot move backwards".into());
        }
        if matches!(
            self.current_stage,
            EconomicActionStage::Completed | EconomicActionStage::Terminated
        ) {
            return Err("Terminal lifecycle cannot receive further revisions".into());
        }

        match revision.kind {
            EconomicActionChangeKind::ScopeAmendment => {
                if revision.predecessor_scope_id.as_deref() != Some(self.active_scope_id.as_str()) {
                    return Err("Scope amendment predecessor does not match active scope".into());
                }
                if revision.scope_fingerprint == self.active_scope_fingerprint {
                    return Err("Scope amendment must change the scope fingerprint".into());
                }
            }
            _ => {
                if revision.scope_id != self.active_scope_id {
                    return Err(
                        "Lifecycle scope cannot change without an explicit scope amendment"
                            .into(),
                    );
                }
                if revision.scope_fingerprint != self.active_scope_fingerprint {
                    return Err(
                        "Lifecycle scope contents cannot change without an explicit scope amendment"
                            .into(),
                    );
                }
            }
        }

        if !stage_transition_allowed(self.current_stage, revision.stage) {
            return Err(format!(
                "Invalid lifecycle transition: {:?} -> {:?}",
                self.current_stage, revision.stage
            ));
        }

        self.current_revision_id = revision.revision_id.clone();
        self.current_stage = revision.stage;
        if revision.kind == EconomicActionChangeKind::ScopeAmendment {
            self.active_scope_id = revision.scope_id.clone();
            self.active_scope_fingerprint = revision.scope_fingerprint.clone();
        }
        self.revisions.push(revision);

        Ok(())
    }
}

/// Return whether a lifecycle stage transition is valid.
pub fn stage_transition_allowed(
    from: EconomicActionStage,
    to: EconomicActionStage,
) -> bool {
    use EconomicActionStage::*;

    match (from, to) {
        (Planning, Planning)
        | (Planning, Tendering)
        | (Planning, Terminated)
        | (Tendering, Tendering)
        | (Tendering, Awarded)
        | (Tendering, Terminated)
        | (Awarded, Awarded)
        | (Awarded, Contracted)
        | (Awarded, Terminated)
        | (Contracted, Contracted)
        | (Contracted, Implementation)
        | (Contracted, Terminated)
        | (Implementation, Implementation)
        | (Implementation, Completed)
        | (Implementation, Terminated) => true,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::integrity_gate::EconomicActionScope;
    use crate::economics::substrate::DistributionPurpose;
    use crate::economics::substrate::SubstrateDimension;

    fn scope(action_ref: &str, scope_id: &str) -> EconomicActionScope {
        EconomicActionScope {
            scope_id: scope_id.into(),
            action_ref: action_ref.into(),
            purpose: DistributionPurpose::Discretionary,
            required_dimensions: vec![SubstrateDimension::Financial],
            policy_ref: "policy:action-scope:v1".into(),
            authority_ref: "authority:dao-1".into(),
            attestation_ref: format!("attestation:{scope_id}"),
            evidence_refs: vec![format!("evidence:{scope_id}")],
            declared_at: 1_000,
        }
    }

    fn start() -> EconomicActionLifecycle {
        EconomicActionLifecycle::start(
            &scope("action:1", "scope:1"),
            "revision:1",
            "authority:dao-1",
            vec!["evidence:revision-1".into()],
            1_000,
        )
        .unwrap()
    }

    #[test]
    fn invalid_deserialized_shape_is_rejected_without_panic() {
        let mut lifecycle = start();
        lifecycle.revisions.clear();
        assert!(lifecycle.validate().is_err());
    }

    #[test]
    fn lifecycle_preserves_action_and_scope_through_normal_progression() {
        let mut lifecycle = start();
        lifecycle
            .record(
                "revision:2",
                EconomicActionStage::Tendering,
                EconomicActionChangeKind::Update,
                &scope("action:1", "scope:1"),
                "authority:dao-1",
                vec!["evidence:revision-2".into()],
                1_100,
            )
            .unwrap();
        lifecycle
            .record(
                "revision:3",
                EconomicActionStage::Awarded,
                EconomicActionChangeKind::Update,
                &scope("action:1", "scope:1"),
                "authority:dao-1",
                vec!["evidence:revision-3".into()],
                1_200,
            )
            .unwrap();

        assert_eq!(lifecycle.action_ref(), "action:1");
        assert_eq!(lifecycle.active_scope_id(), "scope:1");
        assert_eq!(lifecycle.current_stage(), EconomicActionStage::Awarded);
        assert_eq!(lifecycle.revisions().len(), 3);
    }

    #[test]
    fn scope_change_requires_explicit_amendment() {
        let mut lifecycle = start();
        let result = lifecycle.record(
            "revision:2",
            EconomicActionStage::Tendering,
            EconomicActionChangeKind::Update,
            "scope:2",
            "authority:dao-1",
            vec!["evidence:bad-change".into()],
            1_100,
        );

        assert!(result.is_err());
        assert_eq!(lifecycle.active_scope_id(), "scope:1");
        assert_eq!(lifecycle.revisions().len(), 1);
    }

    #[test]
    fn scope_amendment_preserves_action_identity_and_old_scope_history() {
        let mut lifecycle = start();
        lifecycle
            .record(
                "revision:2",
                EconomicActionStage::Contracted,
                EconomicActionChangeKind::Update,
                &scope("action:1", "scope:1"),
                "authority:dao-1",
                vec!["evidence:contract".into()],
                1_100,
            )
            .unwrap();

        let new_scope = scope("action:1", "scope:2");
        lifecycle
            .amend_scope(
                &new_scope,
                "revision:3",
                "authority:independent-1",
                vec!["evidence:amendment".into()],
                1_200,
            )
            .unwrap();

        assert_eq!(lifecycle.action_ref(), "action:1");
        assert_eq!(lifecycle.active_scope_id(), "scope:2");
        assert_eq!(lifecycle.revisions()[1].scope_id, "scope:1");
        assert_eq!(lifecycle.revisions()[2].scope_id, "scope:2");
        assert_eq!(
            lifecycle.revisions()[2].predecessor_scope_id.as_deref(),
            Some("scope:1")
        );
    }

    #[test]
    fn invalid_predecessor_cannot_fork_the_lifecycle() {
        let mut lifecycle = start();
        lifecycle.current_revision_id = "revision:forged".into();

        let result = lifecycle.record(
            "revision:2",
            EconomicActionStage::Tendering,
            EconomicActionChangeKind::Update,
            "scope:1",
            "authority:dao-1",
            vec!["evidence:revision-2".into()],
            1_100,
        );

        assert!(result.is_err());
    }

    #[test]
    fn stage_regression_is_rejected() {
        let mut lifecycle = start();
        lifecycle
            .record(
                "revision:2",
                EconomicActionStage::Tendering,
                EconomicActionChangeKind::Update,
                &scope("action:1", "scope:1"),
                "authority:dao-1",
                vec!["evidence:tender".into()],
                1_100,
            )
            .unwrap();

        let result = lifecycle.record(
            "revision:3",
            EconomicActionStage::Planning,
            EconomicActionChangeKind::Update,
            "scope:1",
            "authority:dao-1",
            vec!["evidence:bad-regression".into()],
            1_200,
        );

        assert!(result.is_err());
    }

    #[test]
    fn terminal_actions_cannot_receive_late_scope_changes() {
        let mut lifecycle = start();
        lifecycle
            .record(
                "revision:2",
                EconomicActionStage::Tendering,
                EconomicActionChangeKind::Update,
                &scope("action:1", "scope:1"),
                "authority:dao-1",
                vec!["evidence:tender".into()],
                1_100,
            )
            .unwrap();
        lifecycle
            .record(
                "revision:3",
                EconomicActionStage::Terminated,
                EconomicActionChangeKind::Termination,
                &scope("action:1", "scope:1"),
                "authority:dao-1",
                vec!["evidence:termination".into()],
                1_200,
            )
            .unwrap();

        let result = lifecycle.amend_scope(
            &scope("action:1", "scope:2"),
            "revision:4",
            "authority:dao-1",
            vec!["evidence:late".into()],
            1_300,
        );

        assert!(result.is_err());
    }

    #[test]
    fn lifecycle_timestamps_are_monotonic() {
        let mut lifecycle = start();
        let result = lifecycle.record(
            "revision:2",
            EconomicActionStage::Tendering,
            EconomicActionChangeKind::Update,
            "scope:1",
            "authority:dao-1",
            vec!["evidence:revision-2".into()],
            999,
        );

        assert!(result.is_err());
    }

    #[test]
    fn scope_must_retain_action_identity_on_amendment() {
        let mut lifecycle = start();
        let result = lifecycle.amend_scope(
            &scope("action:other", "scope:2"),
            "revision:2",
            "authority:dao-1",
            vec!["evidence:bad-action".into()],
            1_100,
        );

        assert!(result.is_err());
    }
}
