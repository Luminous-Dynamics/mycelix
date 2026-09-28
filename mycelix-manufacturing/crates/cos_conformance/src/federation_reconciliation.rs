//! Deterministic branch/reconciliation reference model for heterogeneous federation.
//!
//! Claim ceiling: ReferenceModelOnly.
//! Reconciliation is a new semantic operation. It never rewrites branch history,
//! chooses a winner by arrival order, or converts a projection into authority.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

use super::federation::AuthorityDisposition;

pub const FEDERATION_RECONCILIATION_PROFILE_ID: &str = "INTEGRAL-FED-REF-002";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, PartialOrd, Ord)]
pub enum AuthorityValidity {
    Current,
    Stale,
    Fenced,
    Expired,
    Revoked,
}

impl AuthorityValidity {
    fn is_current(self) -> bool {
        matches!(self, Self::Current)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeliveryClaim {
    pub payload_commitment: String,
    pub origin_node: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticBranch {
    pub branch_id: String,
    pub semantic_environment_root: String,
    pub base_frontier_root: String,
    pub tip_frontier_root: String,
    pub semantic_state_root: String,
    pub provenance_root: String,
    pub invariant_state_root: String,
    pub invariant_conflict: bool,
    pub provenance_conflict: bool,
    pub authority_validity: BTreeMap<String, AuthorityValidity>,
    pub deliveries: BTreeMap<String, DeliveryClaim>,
    pub consumed_capacity: BTreeMap<String, u64>,
    pub authority_claims: BTreeMap<String, AuthorityDisposition>,
    pub closed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BranchCompatibility {
    Equivalent,
    CommutativelyMergeable,
    MergeableWithExplicitResolution,
    Incompatible,
    InsufficientEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, PartialOrd, Ord)]
pub enum ConflictKind {
    IncompleteFrontier,
    PayloadConflict,
    OriginConflict,
    AuthorityConflict,
    CapacityDoubleSpend,
    InvariantConflict,
    ProvenanceConflict,
    EnvironmentConflict,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReconciliationConflict {
    pub kind: ConflictKind,
    pub subject: String,
    pub branch_a_value: String,
    pub branch_b_value: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReconciliationCandidate {
    pub reconciliation_id: String,
    pub base_frontier_root: String,
    pub branch_a_id: String,
    pub branch_b_id: String,
    pub compatibility: BranchCompatibility,
    pub conflicts: Vec<ReconciliationConflict>,
    pub proposed_resolution_root: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReconciliationAction {
    MergeUnion,
    PreserveConflict,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReconciliationOutcome {
    Merged,
    Unresolved,
    Rejected,
    Duplicate,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReconciliationReceipt {
    pub reconciliation_id: String,
    pub branch_a_id: String,
    pub branch_b_id: String,
    pub candidate_compatibility: BranchCompatibility,
    pub proposed_resolution_root: Option<String>,
    pub action: ReconciliationAction,
    pub outcome: ReconciliationOutcome,
    pub resulting_state_root: Option<String>,
    pub conflict_count: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReconciliationState {
    pub branches: BTreeMap<String, SemanticBranch>,
    pub reconciliations: BTreeMap<String, ReconciliationReceipt>,
}

impl ReconciliationState {
    pub fn new(branches: impl IntoIterator<Item = SemanticBranch>) -> Self {
        let branches = branches
            .into_iter()
            .map(|branch| (branch.branch_id.clone(), branch))
            .collect();
        Self {
            branches,
            reconciliations: BTreeMap::new(),
        }
    }
}

fn sorted_conflicts(
    mut conflicts: Vec<ReconciliationConflict>,
) -> Vec<ReconciliationConflict> {
    conflicts.sort_by(|a, b| {
        (a.kind, &a.subject, &a.branch_a_value, &a.branch_b_value).cmp(&(
            b.kind,
            &b.subject,
            &b.branch_a_value,
            &b.branch_b_value,
        ))
    });
    conflicts
}

pub fn classify_branches(
    branch_a: &SemanticBranch,
    branch_b: &SemanticBranch,
) -> (BranchCompatibility, Vec<ReconciliationConflict>) {
    let mut conflicts = Vec::new();

    if !branch_a.closed || !branch_b.closed {
        conflicts.push(ReconciliationConflict {
            kind: ConflictKind::IncompleteFrontier,
            subject: "frontier".into(),
            branch_a_value: branch_a.tip_frontier_root.clone(),
            branch_b_value: branch_b.tip_frontier_root.clone(),
        });
    }

    if branch_a.semantic_environment_root != branch_b.semantic_environment_root {
        conflicts.push(ReconciliationConflict {
            kind: ConflictKind::EnvironmentConflict,
            subject: "semantic_environment".into(),
            branch_a_value: branch_a.semantic_environment_root.clone(),
            branch_b_value: branch_b.semantic_environment_root.clone(),
        });
    }

    if branch_a.base_frontier_root != branch_b.base_frontier_root {
        conflicts.push(ReconciliationConflict {
            kind: ConflictKind::IncompleteFrontier,
            subject: "base_frontier".into(),
            branch_a_value: branch_a.base_frontier_root.clone(),
            branch_b_value: branch_b.base_frontier_root.clone(),
        });
    }

    for (delivery_id, claim_a) in &branch_a.deliveries {
        if let Some(claim_b) = branch_b.deliveries.get(delivery_id) {
            if claim_a.payload_commitment != claim_b.payload_commitment {
                conflicts.push(ReconciliationConflict {
                    kind: ConflictKind::PayloadConflict,
                    subject: delivery_id.clone(),
                    branch_a_value: claim_a.payload_commitment.clone(),
                    branch_b_value: claim_b.payload_commitment.clone(),
                });
            }
            if claim_a.origin_node != claim_b.origin_node {
                conflicts.push(ReconciliationConflict {
                    kind: ConflictKind::OriginConflict,
                    subject: delivery_id.clone(),
                    branch_a_value: claim_a.origin_node.clone(),
                    branch_b_value: claim_b.origin_node.clone(),
                });
            }
        }
    }

    for (capacity_id, amount_a) in &branch_a.consumed_capacity {
        if let Some(amount_b) = branch_b.consumed_capacity.get(capacity_id) {
            if *amount_a > 0 && *amount_b > 0 {
                conflicts.push(ReconciliationConflict {
                    kind: ConflictKind::CapacityDoubleSpend,
                    subject: capacity_id.clone(),
                    branch_a_value: amount_a.to_string(),
                    branch_b_value: amount_b.to_string(),
                });
            }
        }
    }

    for (claim_id, authority_a) in &branch_a.authority_claims {
        if let Some(authority_b) = branch_b.authority_claims.get(claim_id) {
            if authority_a != authority_b {
                conflicts.push(ReconciliationConflict {
                    kind: ConflictKind::AuthorityConflict,
                    subject: claim_id.clone(),
                    branch_a_value: format!("{authority_a:?}"),
                    branch_b_value: format!("{authority_b:?}"),
                });
            }
        }
    }

    for (claim_id, validity_a) in &branch_a.authority_validity {
        if let Some(validity_b) = branch_b.authority_validity.get(claim_id) {
            if validity_a != validity_b {
                conflicts.push(ReconciliationConflict {
                    kind: ConflictKind::AuthorityConflict,
                    subject: format!("{claim_id}:validity"),
                    branch_a_value: format!("{validity_a:?}"),
                    branch_b_value: format!("{validity_b:?}"),
                });
            }
        }
    }

    if branch_a.invariant_conflict || branch_b.invariant_conflict {
        conflicts.push(ReconciliationConflict {
            kind: ConflictKind::InvariantConflict,
            subject: "invariant_state".into(),
            branch_a_value: branch_a.invariant_state_root.clone(),
            branch_b_value: branch_b.invariant_state_root.clone(),
        });
    }

    if branch_a.provenance_conflict || branch_b.provenance_conflict {
        conflicts.push(ReconciliationConflict {
            kind: ConflictKind::ProvenanceConflict,
            subject: "provenance".into(),
            branch_a_value: branch_a.provenance_root.clone(),
            branch_b_value: branch_b.provenance_root.clone(),
        });
    }

    let conflicts = sorted_conflicts(conflicts);

    if !branch_a.closed || !branch_b.closed {
        return (BranchCompatibility::InsufficientEvidence, conflicts);
    }

    if branch_a.semantic_environment_root != branch_b.semantic_environment_root
        || branch_a.base_frontier_root != branch_b.base_frontier_root
    {
        return (BranchCompatibility::Incompatible, conflicts);
    }

    let hard_conflict = conflicts.iter().any(|conflict| {
        matches!(
            conflict.kind,
            ConflictKind::PayloadConflict
                | ConflictKind::OriginConflict
                | ConflictKind::AuthorityConflict
                | ConflictKind::CapacityDoubleSpend
                | ConflictKind::InvariantConflict
                | ConflictKind::EnvironmentConflict
        )
    });

    if hard_conflict {
        return (BranchCompatibility::Incompatible, conflicts);
    }

    if branch_a.semantic_state_root == branch_b.semantic_state_root && conflicts.is_empty() {
        return (BranchCompatibility::Equivalent, conflicts);
    }

    if conflicts.is_empty() {
        (BranchCompatibility::CommutativelyMergeable, conflicts)
    } else {
        (
            BranchCompatibility::MergeableWithExplicitResolution,
            conflicts,
        )
    }
}

pub fn candidate(
    reconciliation_id: impl Into<String>,
    branch_a: &SemanticBranch,
    branch_b: &SemanticBranch,
    proposed_resolution_root: Option<String>,
) -> ReconciliationCandidate {
    let (compatibility, conflicts) = classify_branches(branch_a, branch_b);
    ReconciliationCandidate {
        reconciliation_id: reconciliation_id.into(),
        base_frontier_root: branch_a.base_frontier_root.clone(),
        branch_a_id: branch_a.branch_id.clone(),
        branch_b_id: branch_b.branch_id.clone(),
        compatibility,
        conflicts,
        proposed_resolution_root,
    }
}

pub fn reconcile(
    state: &mut ReconciliationState,
    candidate: &ReconciliationCandidate,
    action: ReconciliationAction,
) -> ReconciliationReceipt {
    if let Some(existing) = state.reconciliations.get(&candidate.reconciliation_id) {
        if existing.branch_a_id == candidate.branch_a_id
            && existing.branch_b_id == candidate.branch_b_id
            && existing.candidate_compatibility == candidate.compatibility
            && existing.proposed_resolution_root == candidate.proposed_resolution_root
            && existing.action == action
        {
            return ReconciliationReceipt {
                outcome: ReconciliationOutcome::Duplicate,
                ..existing.clone()
            };
        }
        return ReconciliationReceipt {
            reconciliation_id: candidate.reconciliation_id.clone(),
            branch_a_id: candidate.branch_a_id.clone(),
            branch_b_id: candidate.branch_b_id.clone(),
            candidate_compatibility: candidate.compatibility,
            proposed_resolution_root: candidate.proposed_resolution_root.clone(),
            action,
            outcome: ReconciliationOutcome::Rejected,
            resulting_state_root: None,
            conflict_count: candidate.conflicts.len(),
        };
    }

    let Some(branch_a) = state.branches.get(&candidate.branch_a_id).cloned() else {
        return rejected(candidate, action);
    };
    let Some(branch_b) = state.branches.get(&candidate.branch_b_id).cloned() else {
        return rejected(candidate, action);
    };

    let (actual_compatibility, actual_conflicts) = classify_branches(&branch_a, &branch_b);
    if actual_compatibility != candidate.compatibility
        || actual_conflicts != candidate.conflicts
    {
        return rejected(candidate, action);
    }

    let receipt = match action {
        ReconciliationAction::MergeUnion => {
            if candidate.compatibility != BranchCompatibility::CommutativelyMergeable {
                rejected(candidate, action)
            } else {
                let mut merged = branch_a.clone();
                merged.branch_id = format!(
                    "{}+{}",
                    branch_a.branch_id, branch_b.branch_id
                );
                merged.tip_frontier_root = format!(
                    "merge:{}:{}",
                    branch_a.tip_frontier_root, branch_b.tip_frontier_root
                );
                merged.semantic_state_root = format!(
                    "union:{}:{}",
                    branch_a.semantic_state_root, branch_b.semantic_state_root
                );
                merged.provenance_root = format!(
                    "union:{}:{}",
                    branch_a.provenance_root, branch_b.provenance_root
                );
                merged.invariant_state_root = format!(
                    "union:{}:{}",
                    branch_a.invariant_state_root, branch_b.invariant_state_root
                );
                merged.deliveries.extend(branch_b.deliveries.clone());
                merged.consumed_capacity.extend(branch_b.consumed_capacity.clone());
                merged.authority_claims.extend(branch_b.authority_claims.clone());
                merged
                    .authority_validity
                    .extend(branch_b.authority_validity.clone());
                state.branches.insert(merged.branch_id.clone(), merged.clone());

                ReconciliationReceipt {
                    reconciliation_id: candidate.reconciliation_id.clone(),
                    branch_a_id: candidate.branch_a_id.clone(),
                    branch_b_id: candidate.branch_b_id.clone(),
                    candidate_compatibility: candidate.compatibility,
                    proposed_resolution_root: candidate.proposed_resolution_root.clone(),
                    action,
                    outcome: ReconciliationOutcome::Merged,
                    resulting_state_root: Some(merged.semantic_state_root),
                    conflict_count: 0,
                }
            }
        }
        ReconciliationAction::PreserveConflict => ReconciliationReceipt {
            reconciliation_id: candidate.reconciliation_id.clone(),
            branch_a_id: candidate.branch_a_id.clone(),
            branch_b_id: candidate.branch_b_id.clone(),
            candidate_compatibility: candidate.compatibility,
            proposed_resolution_root: candidate.proposed_resolution_root.clone(),
            action,
            outcome: ReconciliationOutcome::Unresolved,
            resulting_state_root: None,
            conflict_count: candidate.conflicts.len(),
        },
    };

    state
        .reconciliations
        .insert(candidate.reconciliation_id.clone(), receipt.clone());
    receipt
}

fn rejected(
    candidate: &ReconciliationCandidate,
    action: ReconciliationAction,
) -> ReconciliationReceipt {
    ReconciliationReceipt {
        reconciliation_id: candidate.reconciliation_id.clone(),
        branch_a_id: candidate.branch_a_id.clone(),
        branch_b_id: candidate.branch_b_id.clone(),
        candidate_compatibility: candidate.compatibility,
        proposed_resolution_root: candidate.proposed_resolution_root.clone(),
        action,
        outcome: ReconciliationOutcome::Rejected,
        resulting_state_root: None,
        conflict_count: candidate.conflicts.len(),
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BranchCockpitProjection {
    pub branch_id: String,
    pub base_frontier_root: String,
    pub tip_frontier_root: String,
    pub compatibility: BranchCompatibility,
    pub conflict_count: usize,
    pub local_authority_count: usize,
    pub foreign_evidence_count: usize,
    pub delegated_authority_count: usize,
    pub noncurrent_authority_count: usize,
    pub can_be_normative: bool,
}

pub fn cockpit_projection(
    branch: &SemanticBranch,
    compatibility: BranchCompatibility,
    conflicts: &[ReconciliationConflict],
) -> BranchCockpitProjection {
    let local_authority_count = branch
        .authority_claims
        .values()
        .filter(|value| matches!(value, AuthorityDisposition::LocalAuthority))
        .count();
    let foreign_evidence_count = branch
        .authority_claims
        .values()
        .filter(|value| {
            matches!(
                value,
                AuthorityDisposition::ForeignEvidence
                    | AuthorityDisposition::RecognizedForeignEvidence
            )
        })
        .count();
    let delegated_authority_count = branch
        .authority_claims
        .iter()
        .filter(|(claim_id, value)| {
            matches!(value, AuthorityDisposition::ExplicitDelegatedAuthority)
                && branch
                    .authority_validity
                    .get(*claim_id)
                    .copied()
                    .unwrap_or(AuthorityValidity::Current)
                    .is_current()
        })
        .count();
    let noncurrent_authority_count = branch
        .authority_validity
        .values()
        .filter(|value| !value.is_current())
        .count();

    BranchCockpitProjection {
        branch_id: branch.branch_id.clone(),
        base_frontier_root: branch.base_frontier_root.clone(),
        tip_frontier_root: branch.tip_frontier_root.clone(),
        compatibility,
        conflict_count: conflicts.len(),
        local_authority_count,
        foreign_evidence_count,
        delegated_authority_count,
        noncurrent_authority_count,
        can_be_normative: branch.closed
            && conflicts.is_empty()
            && matches!(
                compatibility,
                BranchCompatibility::Equivalent
                    | BranchCompatibility::CommutativelyMergeable
            ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn branch(
        id: &str,
        state_root: &str,
        delivery_id: &str,
        payload: &str,
        capacity_id: &str,
        authority_id: &str,
    ) -> SemanticBranch {
        SemanticBranch {
            branch_id: id.into(),
            semantic_environment_root: "env-1".into(),
            base_frontier_root: "base-1".into(),
            tip_frontier_root: format!("tip-{id}"),
            semantic_state_root: state_root.into(),
            provenance_root: format!("prov-{id}"),
            invariant_state_root: format!("inv-{id}"),
            invariant_conflict: false,
            provenance_conflict: false,
            authority_validity: BTreeMap::from([(
                authority_id.into(),
                AuthorityValidity::Current,
            )]),
            deliveries: BTreeMap::from([(
                delivery_id.into(),
                DeliveryClaim {
                    payload_commitment: payload.into(),
                    origin_node: format!("origin-{id}"),
                },
            )]),
            consumed_capacity: BTreeMap::from([(capacity_id.into(), 1)]),
            authority_claims: BTreeMap::from([(
                authority_id.into(),
                AuthorityDisposition::LocalAuthority,
            )]),
            closed: true,
        }
    }

    #[test]
    fn disjoint_closed_branches_are_commutatively_mergeable() {
        let a = branch("a", "state-a", "delivery-a", "payload-a", "cap-a", "auth-a");
        let b = branch("b", "state-b", "delivery-b", "payload-b", "cap-b", "auth-b");

        let (compatibility, conflicts) = classify_branches(&a, &b);
        assert_eq!(compatibility, BranchCompatibility::CommutativelyMergeable);
        assert!(conflicts.is_empty());
    }

    #[test]
    fn same_delivery_with_different_payload_is_incompatible() {
        let a = branch("a", "state-a", "delivery-x", "payload-a", "cap-a", "auth-a");
        let b = branch("b", "state-b", "delivery-x", "payload-b", "cap-b", "auth-b");

        let (compatibility, conflicts) = classify_branches(&a, &b);
        assert_eq!(compatibility, BranchCompatibility::Incompatible);
        assert!(conflicts
            .iter()
            .any(|conflict| conflict.kind == ConflictKind::PayloadConflict));
    }

    #[test]
    fn same_capacity_consumption_is_never_union_merged() {
        let a = branch("a", "state-a", "delivery-a", "payload-a", "cap-x", "auth-a");
        let b = branch("b", "state-b", "delivery-b", "payload-b", "cap-x", "auth-b");

        let (compatibility, conflicts) = classify_branches(&a, &b);
        assert_eq!(compatibility, BranchCompatibility::Incompatible);
        assert!(conflicts
            .iter()
            .any(|conflict| conflict.kind == ConflictKind::CapacityDoubleSpend));
    }

    #[test]
    fn stale_authority_cannot_be_revived_by_reconciliation() {
        let a = branch("a", "state-a", "delivery-a", "payload-a", "cap-a", "auth-x");
        let mut b = branch("b", "state-b", "delivery-b", "payload-b", "cap-b", "auth-x");
        b.authority_validity
            .insert("auth-x".into(), AuthorityValidity::Stale);

        let (compatibility, conflicts) = classify_branches(&a, &b);
        assert_eq!(compatibility, BranchCompatibility::Incompatible);
        assert!(conflicts.iter().any(|conflict| {
            conflict.kind == ConflictKind::AuthorityConflict
                && conflict.subject == "auth-x:validity"
        }));
    }

    #[test]
    fn incomplete_branch_is_insufficient_evidence() {
        let mut a = branch("a", "state-a", "delivery-a", "payload-a", "cap-a", "auth-a");
        let b = branch("b", "state-b", "delivery-b", "payload-b", "cap-b", "auth-b");
        a.closed = false;

        let (compatibility, conflicts) = classify_branches(&a, &b);
        assert_eq!(compatibility, BranchCompatibility::InsufficientEvidence);
        assert!(conflicts
            .iter()
            .any(|conflict| conflict.kind == ConflictKind::IncompleteFrontier));
    }

    #[test]
    fn conflicting_provenance_requires_explicit_resolution() {
        let mut a = branch("a", "state-a", "delivery-a", "payload-a", "cap-a", "auth-a");
        let mut b = branch("b", "state-b", "delivery-b", "payload-b", "cap-b", "auth-b");
        a.provenance_root = "prov-shared".into();
        b.provenance_root = "prov-other".into();
        a.provenance_conflict = true;
        let (compatibility, conflicts) = classify_branches(&a, &b);
        assert_eq!(
            compatibility,
            BranchCompatibility::MergeableWithExplicitResolution
        );
        assert!(conflicts
            .iter()
            .any(|conflict| conflict.kind == ConflictKind::ProvenanceConflict));
    }

    #[test]
    fn merge_is_a_new_branch_and_preserves_inputs() {
        let a = branch("a", "state-a", "delivery-a", "payload-a", "cap-a", "auth-a");
        let b = branch("b", "state-b", "delivery-b", "payload-b", "cap-b", "auth-b");
        let candidate = candidate("rec-1", &a, &b, None);
        let mut state = ReconciliationState::new([a.clone(), b.clone()]);

        let receipt = reconcile(
            &mut state,
            &candidate,
            ReconciliationAction::MergeUnion,
        );
        assert_eq!(receipt.outcome, ReconciliationOutcome::Merged);
        assert!(state.branches.contains_key("a"));
        assert!(state.branches.contains_key("b"));
        assert!(state.branches.contains_key("a+b"));
        assert_eq!(state.branches["a"].semantic_state_root, a.semantic_state_root);
        assert_eq!(state.branches["b"].semantic_state_root, b.semantic_state_root);
    }

    #[test]
    fn merge_rejects_incompatible_authority_conflict() {
        let mut a = branch("a", "state-a", "delivery-a", "payload-a", "cap-a", "auth-x");
        let mut b = branch("b", "state-b", "delivery-b", "payload-b", "cap-b", "auth-x");
        b.authority_claims.insert(
            "auth-x".into(),
            AuthorityDisposition::ExplicitDelegatedAuthority,
        );

        let candidate = candidate("rec-auth", &a, &b, None);
        let mut state = ReconciliationState::new([a.clone(), b.clone()]);
        let receipt = reconcile(
            &mut state,
            &candidate,
            ReconciliationAction::MergeUnion,
        );
        assert_eq!(receipt.outcome, ReconciliationOutcome::Rejected);
        assert_eq!(state.branches.len(), 2);
    }

    #[test]
    fn preserve_conflict_does_not_mint_authority() {
        let a = branch("a", "state-a", "delivery-a", "payload-a", "cap-a", "auth-a");
        let mut b = branch("b", "state-b", "delivery-b", "payload-b", "cap-b", "auth-b");
        b.provenance_root = "prov-conflict".into();
        let candidate = candidate("rec-conflict", &a, &b, Some("resolution-proposal-1".into()));
        let mut state = ReconciliationState::new([a, b]);

        let receipt = reconcile(
            &mut state,
            &candidate,
            ReconciliationAction::PreserveConflict,
        );
        assert_eq!(receipt.outcome, ReconciliationOutcome::Unresolved);
        assert_eq!(state.branches.len(), 2);
        assert_eq!(state.reconciliations.len(), 1);
    }

    #[test]
    fn same_reconciliation_is_idempotent_but_changed_inputs_are_rejected() {
        let a = branch("a", "state-a", "delivery-a", "payload-a", "cap-a", "auth-a");
        let b = branch("b", "state-b", "delivery-b", "payload-b", "cap-b", "auth-b");
        let candidate = candidate("rec-idempotent", &a, &b, None);
        let mut state = ReconciliationState::new([a.clone(), b.clone()]);

        let first = reconcile(
            &mut state,
            &candidate,
            ReconciliationAction::MergeUnion,
        );
        assert_eq!(first.outcome, ReconciliationOutcome::Merged);

        let duplicate = reconcile(
            &mut state,
            &candidate,
            ReconciliationAction::MergeUnion,
        );
        assert_eq!(duplicate.outcome, ReconciliationOutcome::Duplicate);

        let mut changed = candidate.clone();
        changed.proposed_resolution_root = Some("different".into());
        let rejected = reconcile(
            &mut state,
            &changed,
            ReconciliationAction::MergeUnion,
        );
        assert_eq!(rejected.outcome, ReconciliationOutcome::Rejected);
    }

    #[test]
    fn cockpit_projection_never_turns_conflict_into_normative_state() {
        let a = branch("a", "state-a", "delivery-a", "payload-a", "cap-a", "auth-a");
        let b = branch("b", "state-b", "delivery-b", "payload-b", "cap-b", "auth-b");
        let candidate = candidate("rec-cockpit", &a, &b, None);
        let conflicts = vec![ReconciliationConflict {
            kind: ConflictKind::ProvenanceConflict,
            subject: "provenance".into(),
            branch_a_value: "a".into(),
            branch_b_value: "b".into(),
        }];

        let projection = cockpit_projection(
            &a,
            BranchCompatibility::MergeableWithExplicitResolution,
            &conflicts,
        );
        assert_eq!(projection.conflict_count, 1);
        assert!(!projection.can_be_normative);
        assert_eq!(projection.local_authority_count, 1);
    }
}
