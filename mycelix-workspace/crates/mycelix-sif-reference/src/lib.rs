// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Reference semantics for Sovereign Intelligence Fabric (SIF) v0.1.
//!
//! This crate is deliberately dependency-light and in-memory. It demonstrates the
//! constitutional behavior before Holochain/storage integration: revoked principals
//! and exhausted budgets are denied, *every* attempt can be receipted, delay grants
//! must enter through an explicit verification gate, and missing/expired delays fail
//! open to subject notification.

use std::collections::{HashMap, HashSet};

use mycelix_sif_protocol::{
    Commitment, DelayedNotificationGrant, DisclosureSummary, ProofBinding, ProvenanceBinding,
    QueryCapability, QueryDecision, SubjectAccessReceipt, UnixMillis, WitnessReceipt,
};

/// Machine-readable reason for a reference policy decision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecisionReason {
    Allowed,
    CapabilityNotYetValid,
    CapabilityExpired,
    PolicyMismatch,
    RequesterRevoked,
    DisclosureNotAllowed,
    BudgetNotConfigured,
    BudgetExhausted,
}

/// Result of evaluating one query capability.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PolicyEvaluation {
    pub decision: QueryDecision,
    pub reason: DecisionReason,
    pub budget_before: Option<u64>,
    pub budget_after: Option<u64>,
    pub budget_limit: Option<u64>,
}

impl PolicyEvaluation {
    fn denied(reason: DecisionReason) -> Self {
        Self {
            decision: QueryDecision::Denied,
            reason,
            budget_before: None,
            budget_after: None,
            budget_limit: None,
        }
    }
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
struct BudgetKey {
    requester: Commitment,
    purpose: String,
    class: String,
}

impl BudgetKey {
    fn from_capability(capability: &QueryCapability) -> Self {
        Self {
            requester: capability.requester_commitment,
            purpose: capability.purpose.clone(),
            class: capability.budget_class.clone(),
        }
    }
}

/// In-memory reference policy engine.
///
/// It is intentionally not a final distributed authorization service. Mycelix
/// identity/governance integrations should supply durable revocation and policy state.
#[derive(Debug)]
pub struct ReferencePolicyEngine {
    policy_commitment: Commitment,
    revoked_requesters: HashSet<Commitment>,
    budget_limits: HashMap<BudgetKey, u64>,
    budget_used: HashMap<BudgetKey, u64>,
}

impl ReferencePolicyEngine {
    pub fn new(policy_commitment: Commitment) -> Self {
        Self {
            policy_commitment,
            revoked_requesters: HashSet::new(),
            budget_limits: HashMap::new(),
            budget_used: HashMap::new(),
        }
    }

    pub fn revoke_requester(&mut self, requester: Commitment) {
        self.revoked_requesters.insert(requester);
    }

    pub fn restore_requester(&mut self, requester: Commitment) {
        self.revoked_requesters.remove(&requester);
    }

    pub fn set_budget_limit(&mut self, capability: &QueryCapability, limit: u64) {
        self.budget_limits
            .insert(BudgetKey::from_capability(capability), limit);
    }

    pub fn budget_used(&self, capability: &QueryCapability) -> Option<u64> {
        let key = BudgetKey::from_capability(capability);
        self.budget_limits
            .contains_key(&key)
            .then(|| self.budget_used.get(&key).copied().unwrap_or(0))
    }

    /// Evaluate one attempt and consume one unit of budget only when authorization
    /// reaches the budget gate and remains within the configured limit.
    pub fn evaluate(
        &mut self,
        capability: &QueryCapability,
        now_ms: UnixMillis,
        requested_disclosures: &[String],
    ) -> PolicyEvaluation {
        if now_ms < capability.valid_from_ms {
            return PolicyEvaluation::denied(DecisionReason::CapabilityNotYetValid);
        }
        if now_ms > capability.valid_until_ms {
            return PolicyEvaluation::denied(DecisionReason::CapabilityExpired);
        }
        if capability.policy_commitment != self.policy_commitment {
            return PolicyEvaluation::denied(DecisionReason::PolicyMismatch);
        }
        if self
            .revoked_requesters
            .contains(&capability.requester_commitment)
        {
            return PolicyEvaluation::denied(DecisionReason::RequesterRevoked);
        }
        if !requested_disclosures
            .iter()
            .all(|requested| capability.allowed_disclosures.iter().any(|a| a == requested))
        {
            return PolicyEvaluation::denied(DecisionReason::DisclosureNotAllowed);
        }

        let key = BudgetKey::from_capability(capability);
        let Some(limit) = self.budget_limits.get(&key).copied() else {
            return PolicyEvaluation::denied(DecisionReason::BudgetNotConfigured);
        };
        let before = self.budget_used.get(&key).copied().unwrap_or(0);
        if before >= limit {
            return PolicyEvaluation {
                decision: QueryDecision::Denied,
                reason: DecisionReason::BudgetExhausted,
                budget_before: Some(before),
                budget_after: Some(before),
                budget_limit: Some(limit),
            };
        }

        let after = before + 1;
        self.budget_used.insert(key, after);
        PolicyEvaluation {
            decision: QueryDecision::Allowed,
            reason: DecisionReason::Allowed,
            budget_before: Some(before),
            budget_after: Some(after),
            budget_limit: Some(limit),
        }
    }
}

/// Error returned when a delayed-notification grant has not passed the external
/// authorization-verification gate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DelayRegistrationError {
    StructurallyInvalid,
    AuthorizationNotVerified,
}

/// Append-only in-memory subject receipt ledger.
#[derive(Debug, Default)]
pub struct ReferenceReceiptLedger {
    receipts: Vec<SubjectAccessReceipt>,
    verified_delays: HashMap<String, DelayedNotificationGrant>,
    witnesses: Vec<WitnessReceipt>,
}

impl ReferenceReceiptLedger {
    pub fn receipts(&self) -> &[SubjectAccessReceipt] {
        &self.receipts
    }

    pub fn witnesses(&self) -> &[WitnessReceipt] {
        &self.witnesses
    }

    /// Register a delay only after the caller's authorization verifier has accepted
    /// the external signature/threshold/warrant artifact.
    ///
    /// The boolean is intentionally explicit in this reference engine; production
    /// adapters should replace it with a typed verifier result.
    pub fn register_verified_delay(
        &mut self,
        grant: DelayedNotificationGrant,
        authorization_verified: bool,
    ) -> Result<(), DelayRegistrationError> {
        if !grant.is_structurally_valid() {
            return Err(DelayRegistrationError::StructurallyInvalid);
        }
        if !authorization_verified {
            return Err(DelayRegistrationError::AuthorizationNotVerified);
        }
        self.verified_delays.insert(grant.grant_id.clone(), grant);
        Ok(())
    }

    /// Create and append an accountability receipt for an evaluated attempt.
    ///
    /// The caller supplies the policy evaluation; both allowed and denied outcomes
    /// are appended. `delay_grant_id` merely references a grant. If that grant was
    /// never successfully registered, subject visibility fails open.
    #[allow(clippy::too_many_arguments)]
    pub fn append_attempt(
        &mut self,
        receipt_id: impl Into<String>,
        capability: &QueryCapability,
        evaluation: &PolicyEvaluation,
        disclosure: DisclosureSummary,
        result_commitment: Option<Commitment>,
        provenance: Option<ProvenanceBinding>,
        proof: Option<ProofBinding>,
        created_at_ms: UnixMillis,
        retention_class: impl Into<String>,
        delay_grant_id: Option<&str>,
    ) -> Commitment {
        let mut receipt = SubjectAccessReceipt::immediate(
            receipt_id,
            capability,
            evaluation.decision,
            disclosure,
            result_commitment,
            provenance,
            proof,
            created_at_ms,
            retention_class,
        );
        if let Some(previous) = self.receipts.last() {
            receipt.previous_receipt_commitment = Some(previous.commitment());
        }
        if let Some(grant_id) = delay_grant_id {
            receipt = receipt.with_delayed_notification(grant_id);
        }
        let commitment = receipt.commitment();
        self.receipts.push(receipt);
        commitment
    }

    /// Subject-visible receipts at `now_ms`.
    pub fn visible_for_subject(
        &self,
        subject: Commitment,
        now_ms: UnixMillis,
    ) -> Vec<&SubjectAccessReceipt> {
        self.receipts
            .iter()
            .filter(|receipt| receipt.subject_commitment == subject)
            .filter(|receipt| {
                let grant = match &receipt.notification {
                    mycelix_sif_protocol::NotificationState::Delayed { grant_id } => {
                        self.verified_delays.get(grant_id)
                    }
                    _ => None,
                };
                receipt.is_subject_visible(now_ms, grant)
            })
            .collect()
    }

    /// Add an independent witness commitment for the latest receipt.
    pub fn witness_latest(
        &mut self,
        witness_commitment: Commitment,
        observed_at_ms: UnixMillis,
        witness_scheme: impl Into<String>,
        attestation_commitment: Commitment,
    ) -> Option<Commitment> {
        let receipt_commitment = self.receipts.last()?.commitment();
        let witness = WitnessReceipt {
            witness_commitment,
            receipt_commitment,
            observed_at_ms,
            witness_scheme: witness_scheme.into(),
            attestation_commitment,
        };
        let commitment = witness.commitment();
        self.witnesses.push(witness);
        Some(commitment)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_sif_protocol::NotificationState;

    fn c(seed: u8) -> Commitment {
        Commitment([seed; 32])
    }

    fn capability() -> QueryCapability {
        QueryCapability {
            query_id: "q-001".into(),
            requester_commitment: c(1),
            organization_commitment: Some(c(2)),
            subject_commitment: c(3),
            purpose: "stolen-vehicle-investigation".into(),
            authority_ref: "case:29418".into(),
            policy_commitment: c(4),
            allowed_disclosures: vec!["plate_match".into()],
            valid_from_ms: 1_000,
            valid_until_ms: 10_000,
            budget_class: "person-linked-standard".into(),
            provenance_required: true,
        }
    }

    fn disclosure() -> DisclosureSummary {
        DisclosureSummary {
            categories: vec!["plate_match".into()],
            fields: vec![],
            raw_evidence_included: false,
        }
    }

    fn delay() -> DelayedNotificationGrant {
        DelayedNotificationGrant {
            grant_id: "delay-001".into(),
            query_id: "q-001".into(),
            authority_ref: "warrant:abc".into(),
            reason_category: "active-kidnapping-investigation".into(),
            approver_commitments: vec![c(20), c(21)],
            issued_at_ms: 1_500,
            reveal_after_ms: 5_000,
            hard_expiry_ms: 6_000,
            policy_commitment: c(4),
            authorization_commitment: c(22),
        }
    }

    #[test]
    fn allowed_lookup_produces_immediately_visible_receipt() {
        let cap = capability();
        let mut policy = ReferencePolicyEngine::new(c(4));
        policy.set_budget_limit(&cap, 10);
        let evaluation = policy.evaluate(&cap, 2_000, &["plate_match".into()]);
        assert_eq!(evaluation.decision, QueryDecision::Allowed);

        let mut ledger = ReferenceReceiptLedger::default();
        ledger.append_attempt(
            "r-001",
            &cap,
            &evaluation,
            disclosure(),
            Some(c(5)),
            None,
            None,
            2_000,
            "accountability-standard",
            None,
        );
        assert_eq!(ledger.visible_for_subject(c(3), 2_000).len(), 1);
    }

    #[test]
    fn revoked_requester_is_denied_and_still_receipted() {
        let cap = capability();
        let mut policy = ReferencePolicyEngine::new(c(4));
        policy.set_budget_limit(&cap, 10);
        policy.revoke_requester(cap.requester_commitment);
        let evaluation = policy.evaluate(&cap, 2_000, &["plate_match".into()]);
        assert_eq!(evaluation.reason, DecisionReason::RequesterRevoked);

        let mut ledger = ReferenceReceiptLedger::default();
        ledger.append_attempt(
            "r-denied",
            &cap,
            &evaluation,
            DisclosureSummary::default(),
            None,
            None,
            None,
            2_000,
            "accountability-standard",
            None,
        );
        let visible = ledger.visible_for_subject(c(3), 2_000);
        assert_eq!(visible.len(), 1);
        assert_eq!(visible[0].decision, QueryDecision::Denied);
    }

    #[test]
    fn budget_exhaustion_denies_second_attempt() {
        let cap = capability();
        let mut policy = ReferencePolicyEngine::new(c(4));
        policy.set_budget_limit(&cap, 1);

        let first = policy.evaluate(&cap, 2_000, &["plate_match".into()]);
        let second = policy.evaluate(&cap, 2_001, &["plate_match".into()]);
        assert_eq!(first.decision, QueryDecision::Allowed);
        assert_eq!(second.decision, QueryDecision::Denied);
        assert_eq!(second.reason, DecisionReason::BudgetExhausted);
        assert_eq!(second.budget_before, Some(1));
        assert_eq!(second.budget_after, Some(1));
    }

    #[test]
    fn unverified_delay_cannot_hide_receipt() {
        let cap = capability();
        let mut policy = ReferencePolicyEngine::new(c(4));
        policy.set_budget_limit(&cap, 10);
        let evaluation = policy.evaluate(&cap, 2_000, &["plate_match".into()]);

        let mut ledger = ReferenceReceiptLedger::default();
        assert_eq!(
            ledger.register_verified_delay(delay(), false),
            Err(DelayRegistrationError::AuthorizationNotVerified)
        );
        ledger.append_attempt(
            "r-001",
            &cap,
            &evaluation,
            disclosure(),
            Some(c(5)),
            None,
            None,
            2_000,
            "accountability-standard",
            Some("delay-001"),
        );
        assert_eq!(ledger.visible_for_subject(c(3), 2_500).len(), 1);
    }

    #[test]
    fn verified_delay_hides_temporarily_then_expires_into_visibility() {
        let cap = capability();
        let mut policy = ReferencePolicyEngine::new(c(4));
        policy.set_budget_limit(&cap, 10);
        let evaluation = policy.evaluate(&cap, 2_000, &["plate_match".into()]);

        let mut ledger = ReferenceReceiptLedger::default();
        ledger.register_verified_delay(delay(), true).unwrap();
        ledger.append_attempt(
            "r-001",
            &cap,
            &evaluation,
            disclosure(),
            Some(c(5)),
            None,
            None,
            2_000,
            "accountability-standard",
            Some("delay-001"),
        );
        assert!(matches!(
            ledger.receipts()[0].notification,
            NotificationState::Delayed { .. }
        ));
        assert_eq!(ledger.visible_for_subject(c(3), 2_500).len(), 0);
        assert_eq!(ledger.visible_for_subject(c(3), 5_000).len(), 1);
    }

    #[test]
    fn receipt_chain_and_independent_witness_bind_history() {
        let cap = capability();
        let mut policy = ReferencePolicyEngine::new(c(4));
        policy.set_budget_limit(&cap, 2);
        let mut ledger = ReferenceReceiptLedger::default();

        for (id, now) in [("r-1", 2_000), ("r-2", 2_001)] {
            let evaluation = policy.evaluate(&cap, now, &["plate_match".into()]);
            ledger.append_attempt(
                id,
                &cap,
                &evaluation,
                disclosure(),
                Some(c(5)),
                None,
                None,
                now,
                "accountability-standard",
                None,
            );
        }

        let first_commitment = ledger.receipts()[0].commitment();
        assert_eq!(
            ledger.receipts()[1].previous_receipt_commitment,
            Some(first_commitment)
        );

        let witness_commitment = ledger
            .witness_latest(c(30), 2_002, "mycelix-independent-witness-v1", c(31))
            .unwrap();
        assert_ne!(witness_commitment, Commitment::ZERO);
        assert_eq!(ledger.witnesses().len(), 1);
        assert_eq!(
            ledger.witnesses()[0].receipt_commitment,
            ledger.receipts()[1].commitment()
        );
    }
}
