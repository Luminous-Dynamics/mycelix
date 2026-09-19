// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic authority policy for Hearth household automation.
//!
//! This crate evaluates an already-validated automation plan step against an
//! evidence-bearing authority snapshot. It intentionally does not fetch DHT
//! state, call zomes, contact devices, or execute actions.
//!
//! HTH-AUTO-002 policy floor:
//! - restrictions override capability grants,
//! - A3 reversible actions may use an existing Hearth autonomy capability,
//! - A4 consequential actions require a scoped grant, explicit approval, or
//!   household decision; a legacy capability string alone is insufficient,
//! - A5 critical actions are structurally limited by `hearth-automation-types`
//!   to explicit approval / household decision / prohibition,
//! - manual override inhibits A3+ side effects,
//! - approvals are unique by approver and bound to an exact intent,
//! - expired facts never authorize execution.

use hearth_automation_types::{
    AuthorityEvidence, AuthorityRequirement, ConsequenceClass, EntityRef, ExecutionStatus,
    HouseholdMode, InhibitionReason, IntentId, PlanStep, ValidationError,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LegacyCapabilityFact {
    pub profile_ref: String,
    pub capability: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RestrictionFact {
    pub profile_ref: String,
    pub capability: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EntitySelector {
    /// `None` matches any semantic entity kind.
    pub kind: Option<String>,
    /// `None` matches any entity ID within the selected kind.
    pub id: Option<String>,
}

impl EntitySelector {
    pub fn any() -> Self {
        Self {
            kind: None,
            id: None,
        }
    }

    pub fn exact(entity: &EntityRef) -> Self {
        Self {
            kind: Some(entity.kind.clone()),
            id: Some(entity.id.clone()),
        }
    }

    pub fn matches(&self, entity: &EntityRef) -> bool {
        self.kind.as_ref().is_none_or(|kind| kind == &entity.kind)
            && self.id.as_ref().is_none_or(|id| id == &entity.id)
    }

    pub fn validate(&self, path: &str) -> Result<(), PolicyValidationError> {
        if self.kind.as_ref().is_some_and(|kind| kind.trim().is_empty()) {
            return Err(PolicyValidationError::new(
                format!("{path}.kind"),
                "kind must not be empty when specified",
            ));
        }
        if self.id.as_ref().is_some_and(|id| id.trim().is_empty()) {
            return Err(PolicyValidationError::new(
                format!("{path}.id"),
                "id must not be empty when specified",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilityScope {
    /// Empty means any target. Prefer explicit selectors for A4 grants.
    pub targets: Vec<EntitySelector>,
    /// Empty means any semantic operation within the capability.
    pub operations: Vec<String>,
    pub max_consequence: ConsequenceClass,
}

impl CapabilityScope {
    pub fn matches(&self, step: &PlanStep) -> bool {
        step.consequence <= self.max_consequence
            && (self.targets.is_empty()
                || self
                    .targets
                    .iter()
                    .any(|selector| selector.matches(&step.action.target)))
            && (self.operations.is_empty()
                || self
                    .operations
                    .iter()
                    .any(|operation| operation == &step.action.operation))
    }

    pub fn validate(&self, path: &str) -> Result<(), PolicyValidationError> {
        for (index, target) in self.targets.iter().enumerate() {
            target.validate(&format!("{path}.targets[{index}]"))?;
        }
        for (index, operation) in self.operations.iter().enumerate() {
            if operation.trim().is_empty() {
                return Err(PolicyValidationError::new(
                    format!("{path}.operations[{index}]"),
                    "operation must not be empty",
                ));
            }
        }
        Ok(())
    }
}

/// A grant defined by the automation policy layer.
///
/// This is intentionally richer than the current Hearth `AutonomyProfile`
/// string capability list so consequential actions can be constrained without
/// a breaking migration of the autonomy zome.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ScopedCapabilityGrant {
    pub reference: String,
    pub subject_id: String,
    pub capability: String,
    pub scope: CapabilityScope,
    pub valid_from_micros: Option<i64>,
    pub expires_at_micros: Option<i64>,
}

impl ScopedCapabilityGrant {
    pub fn is_valid_at(&self, now_micros: i64) -> bool {
        self.valid_from_micros
            .is_none_or(|valid_from| now_micros >= valid_from)
            && self
                .expires_at_micros
                .is_none_or(|expires| now_micros < expires)
    }

    pub fn validate(&self, path: &str) -> Result<(), PolicyValidationError> {
        require_nonempty(&format!("{path}.reference"), &self.reference)?;
        require_nonempty(&format!("{path}.subject_id"), &self.subject_id)?;
        require_nonempty(&format!("{path}.capability"), &self.capability)?;
        self.scope.validate(&format!("{path}.scope"))?;
        if let (Some(valid_from), Some(expires)) =
            (self.valid_from_micros, self.expires_at_micros)
            && expires <= valid_from
        {
            return Err(PolicyValidationError::new(
                format!("{path}.expires_at_micros"),
                "must be later than valid_from_micros",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ApprovalFact {
    pub reference: String,
    pub intent_id: IntentId,
    pub capability: String,
    pub approver_id: String,
    pub approved_at_micros: i64,
    pub expires_at_micros: Option<i64>,
}

impl ApprovalFact {
    fn is_valid_for(
        &self,
        intent_id: &IntentId,
        capability: &str,
        now_micros: i64,
    ) -> bool {
        &self.intent_id == intent_id
            && self.capability == capability
            && self
                .expires_at_micros
                .is_none_or(|expires| now_micros < expires)
    }

    pub fn validate(&self, path: &str) -> Result<(), PolicyValidationError> {
        require_nonempty(&format!("{path}.reference"), &self.reference)?;
        if self.intent_id.is_empty() {
            return Err(PolicyValidationError::new(
                format!("{path}.intent_id"),
                "intent ID must not be empty",
            ));
        }
        require_nonempty(&format!("{path}.capability"), &self.capability)?;
        require_nonempty(&format!("{path}.approver_id"), &self.approver_id)?;
        if let Some(expires) = self.expires_at_micros
            && expires <= self.approved_at_micros
        {
            return Err(PolicyValidationError::new(
                format!("{path}.expires_at_micros"),
                "must be later than approved_at_micros",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HouseholdDecisionFact {
    pub decision_ref: String,
    pub intent_id: IntentId,
    pub approved: bool,
    pub decided_at_micros: i64,
    pub expires_at_micros: Option<i64>,
}

impl HouseholdDecisionFact {
    fn is_valid_for(
        &self,
        decision_ref: &str,
        intent_id: &IntentId,
        now_micros: i64,
    ) -> bool {
        self.decision_ref == decision_ref
            && &self.intent_id == intent_id
            && self.approved
            && self
                .expires_at_micros
                .is_none_or(|expires| now_micros < expires)
    }

    pub fn validate(&self, path: &str) -> Result<(), PolicyValidationError> {
        require_nonempty(&format!("{path}.decision_ref"), &self.decision_ref)?;
        if self.intent_id.is_empty() {
            return Err(PolicyValidationError::new(
                format!("{path}.intent_id"),
                "intent ID must not be empty",
            ));
        }
        if let Some(expires) = self.expires_at_micros
            && expires <= self.decided_at_micros
        {
            return Err(PolicyValidationError::new(
                format!("{path}.expires_at_micros"),
                "must be later than decided_at_micros",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthoritySnapshot {
    pub actor_id: String,
    pub household_mode: HouseholdMode,
    pub captured_at_micros: i64,
    pub legacy_capabilities: Vec<LegacyCapabilityFact>,
    pub restrictions: Vec<RestrictionFact>,
    pub scoped_grants: Vec<ScopedCapabilityGrant>,
    pub approvals: Vec<ApprovalFact>,
    pub household_decisions: Vec<HouseholdDecisionFact>,
}

impl AuthoritySnapshot {
    pub fn from_legacy_profile(
        actor_id: impl Into<String>,
        household_mode: HouseholdMode,
        profile_ref: impl Into<String>,
        capabilities: impl IntoIterator<Item = String>,
        restrictions: impl IntoIterator<Item = String>,
        captured_at_micros: i64,
    ) -> Self {
        let profile_ref = profile_ref.into();
        Self {
            actor_id: actor_id.into(),
            household_mode,
            captured_at_micros,
            legacy_capabilities: capabilities
                .into_iter()
                .map(|capability| LegacyCapabilityFact {
                    profile_ref: profile_ref.clone(),
                    capability,
                })
                .collect(),
            restrictions: restrictions
                .into_iter()
                .map(|capability| RestrictionFact {
                    profile_ref: profile_ref.clone(),
                    capability,
                })
                .collect(),
            scoped_grants: Vec::new(),
            approvals: Vec::new(),
            household_decisions: Vec::new(),
        }
    }

    pub fn validate_at(&self, now_micros: i64) -> Result<(), PolicyValidationError> {
        require_nonempty("actor_id", &self.actor_id)?;
        if self.captured_at_micros > now_micros {
            return Err(PolicyValidationError::new(
                "captured_at_micros",
                "authority snapshot cannot be from the future",
            ));
        }
        for (index, fact) in self.legacy_capabilities.iter().enumerate() {
            require_nonempty(
                &format!("legacy_capabilities[{index}].profile_ref"),
                &fact.profile_ref,
            )?;
            require_nonempty(
                &format!("legacy_capabilities[{index}].capability"),
                &fact.capability,
            )?;
        }
        for (index, fact) in self.restrictions.iter().enumerate() {
            require_nonempty(
                &format!("restrictions[{index}].profile_ref"),
                &fact.profile_ref,
            )?;
            require_nonempty(
                &format!("restrictions[{index}].capability"),
                &fact.capability,
            )?;
        }
        for (index, grant) in self.scoped_grants.iter().enumerate() {
            grant.validate(&format!("scoped_grants[{index}]"))?;
        }
        for (index, approval) in self.approvals.iter().enumerate() {
            approval.validate(&format!("approvals[{index}]"))?;
        }
        for (index, decision) in self.household_decisions.iter().enumerate() {
            decision.validate(&format!("household_decisions[{index}]"))?;
        }
        Ok(())
    }

    fn is_restricted(&self, capability: &str) -> bool {
        self.restrictions
            .iter()
            .any(|restriction| restriction.capability == capability)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorityDenial {
    InvalidContract {
        path: String,
        message: String,
    },
    InvalidSnapshot {
        path: String,
        message: String,
    },
    Forbidden,
    MissingCapability {
        capability: String,
    },
    RestrictedCapability {
        capability: String,
    },
    ScopedGrantRequired {
        capability: String,
    },
    ScopeMismatch {
        capability: String,
    },
    HouseholdDecisionMissing {
        decision_ref: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PolicyDecision {
    Allowed {
        evidence: AuthorityEvidence,
    },
    ApprovalRequired {
        capability: String,
        approvals_required: u16,
        valid_approvals: u16,
    },
    Denied(AuthorityDenial),
    Inhibited(InhibitionReason),
}

impl PolicyDecision {
    pub fn is_allowed(&self) -> bool {
        matches!(self, Self::Allowed { .. })
    }

    pub fn as_execution_status(&self) -> Option<ExecutionStatus> {
        match self {
            Self::Inhibited(reason) => Some(ExecutionStatus::Inhibited(reason.clone())),
            Self::Denied(reason) => Some(ExecutionStatus::Failed {
                reason: format!("{reason:?}"),
            }),
            Self::Allowed { .. } | Self::ApprovalRequired { .. } => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PolicyValidationError {
    pub path: String,
    pub message: String,
}

impl PolicyValidationError {
    fn new(path: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            path: path.into(),
            message: message.into(),
        }
    }
}

fn require_nonempty(path: &str, value: &str) -> Result<(), PolicyValidationError> {
    if value.trim().is_empty() {
        Err(PolicyValidationError::new(path, "must not be empty"))
    } else {
        Ok(())
    }
}

fn invalid_contract(error: ValidationError) -> PolicyDecision {
    PolicyDecision::Denied(AuthorityDenial::InvalidContract {
        path: error.path,
        message: error.message,
    })
}

fn invalid_snapshot(error: PolicyValidationError) -> PolicyDecision {
    PolicyDecision::Denied(AuthorityDenial::InvalidSnapshot {
        path: error.path,
        message: error.message,
    })
}

fn empty_evidence(now_micros: i64) -> AuthorityEvidence {
    AuthorityEvidence {
        evaluated_at_micros: now_micros,
        capability_grants: Vec::new(),
        approvals: Vec::new(),
        household_decisions: Vec::new(),
    }
}

/// Evaluate one concrete plan step.
///
/// Plans are evaluated at execution time, not only when an intent is created,
/// so expiry, restriction changes, and manual override are observed immediately.
pub fn evaluate_step(
    intent_id: &IntentId,
    step: &PlanStep,
    snapshot: &AuthoritySnapshot,
    now_micros: i64,
) -> PolicyDecision {
    if intent_id.is_empty() {
        return PolicyDecision::Denied(AuthorityDenial::InvalidContract {
            path: "intent_id".into(),
            message: "must not be empty".into(),
        });
    }
    if let Err(error) = step.validate("step") {
        return invalid_contract(error);
    }
    if let Err(error) = snapshot.validate_at(now_micros) {
        return invalid_snapshot(error);
    }

    if snapshot.household_mode == HouseholdMode::ManualOverride
        && step.consequence.requires_execution_authority()
    {
        return PolicyDecision::Inhibited(InhibitionReason::ManualOverride);
    }

    match &step.authority {
        AuthorityRequirement::None => PolicyDecision::Allowed {
            evidence: empty_evidence(now_micros),
        },
        AuthorityRequirement::Forbidden => PolicyDecision::Denied(AuthorityDenial::Forbidden),
        AuthorityRequirement::Capability { capability } => {
            evaluate_capability(step, snapshot, capability, now_micros)
        }
        AuthorityRequirement::ExplicitApproval {
            capability,
            approvals_required,
        } => evaluate_approval(
            intent_id,
            snapshot,
            capability.as_deref().unwrap_or(&step.action.capability),
            *approvals_required,
            now_micros,
        ),
        AuthorityRequirement::HouseholdDecision { decision_ref } => {
            evaluate_household_decision(intent_id, snapshot, decision_ref, now_micros)
        }
    }
}

fn evaluate_capability(
    step: &PlanStep,
    snapshot: &AuthoritySnapshot,
    capability: &str,
    now_micros: i64,
) -> PolicyDecision {
    if snapshot.is_restricted(capability) {
        return PolicyDecision::Denied(AuthorityDenial::RestrictedCapability {
            capability: capability.to_owned(),
        });
    }

    let matching_scoped: Vec<_> = snapshot
        .scoped_grants
        .iter()
        .filter(|grant| {
            grant.subject_id == snapshot.actor_id
                && grant.capability == capability
                && grant.is_valid_at(now_micros)
                && grant.scope.matches(step)
        })
        .collect();

    if step.consequence >= ConsequenceClass::ConsequentialAct {
        if let Some(grant) = matching_scoped.first() {
            let mut evidence = empty_evidence(now_micros);
            evidence.capability_grants.push(grant.reference.clone());
            return PolicyDecision::Allowed { evidence };
        }

        let has_any_scoped = snapshot.scoped_grants.iter().any(|grant| {
            grant.subject_id == snapshot.actor_id
                && grant.capability == capability
                && grant.is_valid_at(now_micros)
        });

        return PolicyDecision::Denied(if has_any_scoped {
            AuthorityDenial::ScopeMismatch {
                capability: capability.to_owned(),
            }
        } else {
            AuthorityDenial::ScopedGrantRequired {
                capability: capability.to_owned(),
            }
        });
    }

    if let Some(grant) = matching_scoped.first() {
        let mut evidence = empty_evidence(now_micros);
        evidence.capability_grants.push(grant.reference.clone());
        return PolicyDecision::Allowed { evidence };
    }

    if let Some(fact) = snapshot
        .legacy_capabilities
        .iter()
        .find(|fact| fact.capability == capability)
    {
        let mut evidence = empty_evidence(now_micros);
        evidence.capability_grants.push(fact.profile_ref.clone());
        return PolicyDecision::Allowed { evidence };
    }

    PolicyDecision::Denied(AuthorityDenial::MissingCapability {
        capability: capability.to_owned(),
    })
}

fn evaluate_approval(
    intent_id: &IntentId,
    snapshot: &AuthoritySnapshot,
    capability: &str,
    approvals_required: u16,
    now_micros: i64,
) -> PolicyDecision {
    if snapshot.is_restricted(capability) {
        return PolicyDecision::Denied(AuthorityDenial::RestrictedCapability {
            capability: capability.to_owned(),
        });
    }

    let mut approvers = BTreeSet::new();
    let mut references = Vec::new();

    for approval in snapshot
        .approvals
        .iter()
        .filter(|approval| approval.is_valid_for(intent_id, capability, now_micros))
    {
        if approvers.insert(approval.approver_id.clone()) {
            references.push(approval.reference.clone());
        }
    }

    let valid = u16::try_from(approvers.len()).unwrap_or(u16::MAX);
    if valid < approvals_required {
        return PolicyDecision::ApprovalRequired {
            capability: capability.to_owned(),
            approvals_required,
            valid_approvals: valid,
        };
    }

    let mut evidence = empty_evidence(now_micros);
    evidence.approvals = references;
    PolicyDecision::Allowed { evidence }
}

fn evaluate_household_decision(
    intent_id: &IntentId,
    snapshot: &AuthoritySnapshot,
    decision_ref: &str,
    now_micros: i64,
) -> PolicyDecision {
    let Some(decision) = snapshot
        .household_decisions
        .iter()
        .find(|decision| decision.is_valid_for(decision_ref, intent_id, now_micros))
    else {
        return PolicyDecision::Denied(AuthorityDenial::HouseholdDecisionMissing {
            decision_ref: decision_ref.to_owned(),
        });
    };

    let mut evidence = empty_evidence(now_micros);
    evidence
        .household_decisions
        .push(decision.decision_ref.clone());
    PolicyDecision::Allowed { evidence }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_automation_types::{
        ActionSpec, AutomationValue, ComparisonOp, ConditionExpr, EvidenceRequirement,
        OutcomeExpectation, OutcomePolicy, Reversibility, RetryPolicy, StateExpectation, StepId,
        UnverifiedDisposition,
    };
    use std::collections::BTreeMap;

    fn target() -> EntityRef {
        EntityRef {
            kind: "door".into(),
            id: "front".into(),
        }
    }

    fn outcome() -> OutcomePolicy {
        OutcomePolicy {
            expectations: vec![OutcomeExpectation::State(StateExpectation {
                subject: target(),
                attribute: "locked".into(),
                op: ComparisonOp::Eq,
                expected: AutomationValue::Bool(true),
            })],
            verify_within_ms: 5_000,
            evidence: EvidenceRequirement::default(),
            on_unverified: UnverifiedDisposition::StopAndNotify,
        }
    }

    fn step(consequence: ConsequenceClass, authority: AuthorityRequirement) -> PlanStep {
        PlanStep {
            id: StepId::from("lock-front"),
            depends_on: Vec::new(),
            preconditions: ConditionExpr::Always,
            action: ActionSpec {
                capability: "home.access.lock".into(),
                target: target(),
                operation: "lock".into(),
                arguments: BTreeMap::new(),
            },
            consequence,
            reversibility: Reversibility::Reversible,
            authority,
            outcome: if consequence.requires_verified_outcome() {
                outcome()
            } else {
                OutcomePolicy {
                    expectations: Vec::new(),
                    verify_within_ms: 0,
                    evidence: EvidenceRequirement::default(),
                    on_unverified: UnverifiedDisposition::StopAndNotify,
                }
            },
            timeout_ms: 5_000,
            retry: RetryPolicy::none(),
            compensation: None,
        }
    }

    fn snapshot(capabilities: &[&str], restrictions: &[&str]) -> AuthoritySnapshot {
        AuthoritySnapshot::from_legacy_profile(
            "agent-a",
            HouseholdMode::Normal,
            "profile-1",
            capabilities.iter().map(|value| (*value).to_owned()),
            restrictions.iter().map(|value| (*value).to_owned()),
            100,
        )
    }

    #[test]
    fn a3_can_use_existing_autonomy_capability() {
        let candidate = step(
            ConsequenceClass::ReversibleAct,
            AuthorityRequirement::Capability {
                capability: "home.access.lock".into(),
            },
        );
        let decision = evaluate_step(
            &IntentId::from("intent-1"),
            &candidate,
            &snapshot(&["home.access.lock"], &[]),
            200,
        );
        assert!(decision.is_allowed());
    }

    #[test]
    fn restriction_overrides_legacy_capability() {
        let candidate = step(
            ConsequenceClass::ReversibleAct,
            AuthorityRequirement::Capability {
                capability: "home.access.lock".into(),
            },
        );
        let decision = evaluate_step(
            &IntentId::from("intent-1"),
            &candidate,
            &snapshot(&["home.access.lock"], &["home.access.lock"]),
            200,
        );
        assert!(matches!(
            decision,
            PolicyDecision::Denied(AuthorityDenial::RestrictedCapability { .. })
        ));
    }

    #[test]
    fn a4_rejects_legacy_string_capability_without_scope() {
        let candidate = step(
            ConsequenceClass::ConsequentialAct,
            AuthorityRequirement::Capability {
                capability: "home.access.lock".into(),
            },
        );
        let decision = evaluate_step(
            &IntentId::from("intent-1"),
            &candidate,
            &snapshot(&["home.access.lock"], &[]),
            200,
        );
        assert!(matches!(
            decision,
            PolicyDecision::Denied(AuthorityDenial::ScopedGrantRequired { .. })
        ));
    }

    #[test]
    fn a4_accepts_matching_scoped_grant() {
        let candidate = step(
            ConsequenceClass::ConsequentialAct,
            AuthorityRequirement::Capability {
                capability: "home.access.lock".into(),
            },
        );
        let mut facts = snapshot(&[], &[]);
        facts.scoped_grants.push(ScopedCapabilityGrant {
            reference: "grant-1".into(),
            subject_id: "agent-a".into(),
            capability: "home.access.lock".into(),
            scope: CapabilityScope {
                targets: vec![EntitySelector::exact(&target())],
                operations: vec!["lock".into()],
                max_consequence: ConsequenceClass::ConsequentialAct,
            },
            valid_from_micros: Some(100),
            expires_at_micros: Some(300),
        });

        let decision = evaluate_step(&IntentId::from("intent-1"), &candidate, &facts, 200);
        assert!(decision.is_allowed());
    }

    #[test]
    fn a4_rejects_scope_mismatch() {
        let candidate = step(
            ConsequenceClass::ConsequentialAct,
            AuthorityRequirement::Capability {
                capability: "home.access.lock".into(),
            },
        );
        let mut facts = snapshot(&[], &[]);
        facts.scoped_grants.push(ScopedCapabilityGrant {
            reference: "grant-1".into(),
            subject_id: "agent-a".into(),
            capability: "home.access.lock".into(),
            scope: CapabilityScope {
                targets: vec![EntitySelector {
                    kind: Some("door".into()),
                    id: Some("back".into()),
                }],
                operations: vec!["lock".into()],
                max_consequence: ConsequenceClass::ConsequentialAct,
            },
            valid_from_micros: None,
            expires_at_micros: None,
        });

        let decision = evaluate_step(&IntentId::from("intent-1"), &candidate, &facts, 200);
        assert!(matches!(
            decision,
            PolicyDecision::Denied(AuthorityDenial::ScopeMismatch { .. })
        ));
    }

    #[test]
    fn approvals_are_unique_by_approver() {
        let candidate = step(
            ConsequenceClass::CriticalAct,
            AuthorityRequirement::ExplicitApproval {
                capability: Some("home.access.lock".into()),
                approvals_required: 2,
            },
        );
        let mut facts = snapshot(&[], &[]);
        facts.approvals.extend([
            ApprovalFact {
                reference: "approval-1".into(),
                intent_id: IntentId::from("intent-1"),
                capability: "home.access.lock".into(),
                approver_id: "guardian-a".into(),
                approved_at_micros: 100,
                expires_at_micros: None,
            },
            ApprovalFact {
                reference: "approval-2".into(),
                intent_id: IntentId::from("intent-1"),
                capability: "home.access.lock".into(),
                approver_id: "guardian-a".into(),
                approved_at_micros: 110,
                expires_at_micros: None,
            },
        ]);

        let decision = evaluate_step(&IntentId::from("intent-1"), &candidate, &facts, 200);
        assert!(matches!(
            decision,
            PolicyDecision::ApprovalRequired {
                approvals_required: 2,
                valid_approvals: 1,
                ..
            }
        ));
    }

    #[test]
    fn expired_approval_does_not_authorize() {
        let candidate = step(
            ConsequenceClass::CriticalAct,
            AuthorityRequirement::ExplicitApproval {
                capability: Some("home.access.lock".into()),
                approvals_required: 1,
            },
        );
        let mut facts = snapshot(&[], &[]);
        facts.approvals.push(ApprovalFact {
            reference: "approval-1".into(),
            intent_id: IntentId::from("intent-1"),
            capability: "home.access.lock".into(),
            approver_id: "guardian-a".into(),
            approved_at_micros: 100,
            expires_at_micros: Some(150),
        });

        let decision = evaluate_step(&IntentId::from("intent-1"), &candidate, &facts, 200);
        assert!(matches!(
            decision,
            PolicyDecision::ApprovalRequired {
                valid_approvals: 0,
                ..
            }
        ));
    }

    #[test]
    fn manual_override_inhibits_side_effects() {
        let candidate = step(
            ConsequenceClass::ReversibleAct,
            AuthorityRequirement::Capability {
                capability: "home.access.lock".into(),
            },
        );
        let mut facts = snapshot(&["home.access.lock"], &[]);
        facts.household_mode = HouseholdMode::ManualOverride;

        let decision = evaluate_step(&IntentId::from("intent-1"), &candidate, &facts, 200);
        assert_eq!(
            decision,
            PolicyDecision::Inhibited(InhibitionReason::ManualOverride)
        );
    }

    #[test]
    fn household_decision_is_bound_to_exact_intent() {
        let candidate = step(
            ConsequenceClass::CriticalAct,
            AuthorityRequirement::HouseholdDecision {
                decision_ref: "decision-1".into(),
            },
        );
        let mut facts = snapshot(&[], &[]);
        facts.household_decisions.push(HouseholdDecisionFact {
            decision_ref: "decision-1".into(),
            intent_id: IntentId::from("different-intent"),
            approved: true,
            decided_at_micros: 100,
            expires_at_micros: None,
        });

        let decision = evaluate_step(&IntentId::from("intent-1"), &candidate, &facts, 200);
        assert!(matches!(
            decision,
            PolicyDecision::Denied(AuthorityDenial::HouseholdDecisionMissing { .. })
        ));
    }

    #[test]
    fn household_decision_authorizes_exact_intent() {
        let candidate = step(
            ConsequenceClass::CriticalAct,
            AuthorityRequirement::HouseholdDecision {
                decision_ref: "decision-1".into(),
            },
        );
        let mut facts = snapshot(&[], &[]);
        facts.household_decisions.push(HouseholdDecisionFact {
            decision_ref: "decision-1".into(),
            intent_id: IntentId::from("intent-1"),
            approved: true,
            decided_at_micros: 100,
            expires_at_micros: Some(300),
        });

        let decision = evaluate_step(&IntentId::from("intent-1"), &candidate, &facts, 200);
        assert!(decision.is_allowed());
    }
}
