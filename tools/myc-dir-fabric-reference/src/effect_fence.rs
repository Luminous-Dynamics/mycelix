//! Resource-enforced revocation fencing for authorization-bound mutations.
//!
//! This is a deterministic reference contract, not a durable fence service.
//! A production effect owner must persist the generation and compare it in
//! the same atomic transaction as the protected mutation. A separate
//! preflight check followed by a later write is not sufficient.

use std::collections::{BTreeMap, BTreeSet};

use crate::effect_inventory::{
    assess_effect_surface_closure, EffectSurfaceAssessment, EffectSurfaceClosureReceipt,
    EffectSurfaceInventory,
};
use crate::receipts::{
    DecisionOutcome, DecisionReceipt, EffectOwnerId, LogicalOperationId,
};
use crate::{ActionId, Digest, RealmId, ResourceId, MAX_IDENTIFIER_BYTES};

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FenceError {
    EmptyIdentifier,
    IdentifierTooLong,
    ZeroDigest,
    GenerationNotMonotonic,
    GenerationExhausted,
    EmptyOwnerInventory,
    DuplicateRequiredOwner,
    DuplicateAcknowledgement,
    UnexpectedOwner,
    ScopeMismatch,
    GenerationMismatch,
    RevocationBindingMismatch,
    InventoryNotStronglyCovered,
    EffectOwnerSetMismatch,
}

impl std::fmt::Display for FenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for FenceError {}

fn checked_text(value: &str) -> Result<(), FenceError> {
    if value.trim().is_empty() {
        return Err(FenceError::EmptyIdentifier);
    }
    if value.len() > MAX_IDENTIFIER_BYTES {
        return Err(FenceError::IdentifierTooLong);
    }
    Ok(())
}

fn append_text(bytes: &mut Vec<u8>, value: &str) -> Result<(), FenceError> {
    checked_text(value)?;
    let length = u16::try_from(value.len()).map_err(|_| FenceError::IdentifierTooLong)?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(value.as_bytes());
    Ok(())
}

/// Exact mutation namespace. Fencing is scoped to a realm, resource and
/// action; it is not a global directory sequence.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct EffectFenceScope {
    realm: RealmId,
    resource: ResourceId,
    action: ActionId,
}

impl EffectFenceScope {
    pub fn new(realm: RealmId, resource: ResourceId, action: ActionId) -> Self {
        Self { realm, resource, action }
    }

    pub fn realm(&self) -> &RealmId {
        &self.realm
    }

    pub fn resource(&self) -> &ResourceId {
        &self.resource
    }

    pub fn action(&self) -> &ActionId {
        &self.action
    }

    fn append_canonical(&self, bytes: &mut Vec<u8>) -> Result<(), FenceError> {
        append_text(bytes, self.realm.as_str())?;
        append_text(bytes, self.resource.as_str())?;
        append_text(bytes, self.action.as_str())
    }
}

/// One effect owner's namespace within an exact resource/action scope.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct EffectFenceDomain {
    scope: EffectFenceScope,
    owner: EffectOwnerId,
}

impl EffectFenceDomain {
    pub fn new(scope: EffectFenceScope, owner: EffectOwnerId) -> Self {
        Self { scope, owner }
    }

    pub fn scope(&self) -> &EffectFenceScope {
        &self.scope
    }

    pub fn owner(&self) -> &EffectOwnerId {
        &self.owner
    }

    fn append_canonical(&self, bytes: &mut Vec<u8>) -> Result<(), FenceError> {
        self.scope.append_canonical(bytes)?;
        append_text(bytes, self.owner.as_str())
    }
}

/// A resource-owner-issued generation. Its canonical bytes are not a
/// signature: callers must authenticate the issuing owner and the command.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectFenceToken {
    domain: EffectFenceDomain,
    generation: u64,
}

impl EffectFenceToken {
    /// Reconstructs a token received from an external protocol. Structural
    /// validation does not authenticate the token or its issuer.
    pub fn from_wire(domain: EffectFenceDomain, generation: u64) -> Self {
        Self { domain, generation }
    }

    pub fn domain(&self) -> &EffectFenceDomain {
        &self.domain
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, FenceError> {
        let mut bytes = b"MYC-DIR-EFFECT-FENCE-TOKEN\0\x01".to_vec();
        self.domain.append_canonical(&mut bytes)?;
        bytes.extend_from_slice(&self.generation.to_be_bytes());
        Ok(bytes)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FenceAssessment {
    Current,
    StaleGeneration,
    FutureOrUnissuedGeneration,
    WrongDomain,
}

/// Pure reference state for one resource owner's durable monotonic fence.
/// Production storage must make installation durable before acknowledging it.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectFenceState {
    domain: EffectFenceDomain,
    generation: u64,
}

impl EffectFenceState {
    /// Generation 1 is the initial reference-model state; 0 is never a valid
    /// production token. This is not a restart constructor: production recovery
    /// must restore the durable monotonic high-water generation, never reset to 1.
    pub fn new(domain: EffectFenceDomain) -> Self {
        Self { domain, generation: 1 }
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn current_token(&self) -> EffectFenceToken {
        EffectFenceToken {
            domain: self.domain.clone(),
            generation: self.generation,
        }
    }

    /// Models a durable install. The caller must not emit an owner
    /// acknowledgement until the new generation survives restart/recovery.
    pub fn install_generation(
        &mut self,
        next_generation: u64,
    ) -> Result<EffectFenceToken, FenceError> {
        if next_generation <= self.generation {
            return Err(FenceError::GenerationNotMonotonic);
        }
        self.generation = next_generation;
        Ok(self.current_token())
    }

    pub fn advance_generation(&mut self) -> Result<EffectFenceToken, FenceError> {
        let next = self
            .generation
            .checked_add(1)
            .ok_or(FenceError::GenerationExhausted)?;
        self.install_generation(next)
    }

    /// This comparison must be performed atomically with mutation commit by
    /// the real resource owner. Calling it as a separate preflight is unsafe.
    pub fn assess(&self, token: &EffectFenceToken) -> FenceAssessment {
        if token.domain != self.domain {
            return FenceAssessment::WrongDomain;
        }
        match token.generation.cmp(&self.generation) {
            std::cmp::Ordering::Less => FenceAssessment::StaleGeneration,
            std::cmp::Ordering::Equal => FenceAssessment::Current,
            std::cmp::Ordering::Greater => FenceAssessment::FutureOrUnissuedGeneration,
        }
    }
}

/// One owner that must install a revocation generation. The per-owner target
/// allows independent resource owners to retain local monotonic sequences.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerFenceTarget {
    owner: EffectOwnerId,
    generation: u64,
}

impl OwnerFenceTarget {
    pub fn new(owner: EffectOwnerId, generation: u64) -> Result<Self, FenceError> {
        if generation == 0 {
            return Err(FenceError::GenerationNotMonotonic);
        }
        Ok(Self { owner, generation })
    }

    pub fn owner(&self) -> &EffectOwnerId {
        &self.owner
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }
}

/// A revocation plan can be constructed only from a structurally bound,
/// independently verified complete effect-surface inventory. Its exact owner
/// set is derived from enumerated routes; caller-supplied targets must match it.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RevocationFencePlan {
    scope: EffectFenceScope,
    revocation_digest: Digest,
    inventory_epoch: u64,
    inventory_manifest_digest: Digest,
    route_set_digest: Digest,
    closure_proof_digest: Digest,
    route_count: usize,
    targets: Vec<OwnerFenceTarget>,
}

impl RevocationFencePlan {
    pub fn from_verified_inventory(
        inventory: &EffectSurfaceInventory,
        closure_receipt: &EffectSurfaceClosureReceipt,
        mut targets: Vec<OwnerFenceTarget>,
    ) -> Result<Self, FenceError> {
        if assess_effect_surface_closure(inventory, closure_receipt)
            != EffectSurfaceAssessment::StrongFenceEligible
        {
            return Err(FenceError::InventoryNotStronglyCovered);
        }
        if targets.is_empty() {
            return Err(FenceError::EmptyOwnerInventory);
        }

        targets.sort_by(|left, right| left.owner.cmp(&right.owner));
        for pair in targets.windows(2) {
            if pair[0].owner == pair[1].owner {
                return Err(FenceError::DuplicateRequiredOwner);
            }
        }

        let expected_owners: BTreeSet<EffectOwnerId> = inventory
            .routes()
            .iter()
            .map(|route| route.owner().clone())
            .collect();
        let supplied_owners: BTreeSet<EffectOwnerId> = targets
            .iter()
            .map(|target| target.owner.clone())
            .collect();
        if expected_owners != supplied_owners {
            return Err(FenceError::EffectOwnerSetMismatch);
        }

        Ok(Self {
            scope: inventory.scope().clone(),
            revocation_digest: *inventory.revocation_digest(),
            inventory_epoch: inventory.inventory_epoch(),
            inventory_manifest_digest: *inventory.inventory_manifest_digest(),
            route_set_digest: *inventory.route_set_digest(),
            closure_proof_digest: *closure_receipt.proof_digest(),
            route_count: inventory.routes().len(),
            targets,
        })
    }

    pub fn targets(&self) -> &[OwnerFenceTarget] {
        &self.targets
    }

    pub fn inventory_epoch(&self) -> u64 {
        self.inventory_epoch
    }

    pub fn route_count(&self) -> usize {
        self.route_count
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, FenceError> {
        let mut bytes = b"MYC-DIR-REVOCATION-FENCE-PLAN\0\x02".to_vec();
        self.scope.append_canonical(&mut bytes)?;
        bytes.extend_from_slice(&self.revocation_digest);
        bytes.extend_from_slice(&self.inventory_epoch.to_be_bytes());
        bytes.extend_from_slice(&self.inventory_manifest_digest);
        bytes.extend_from_slice(&self.route_set_digest);
        bytes.extend_from_slice(&self.closure_proof_digest);
        let route_count =
            u32::try_from(self.route_count).map_err(|_| FenceError::EmptyOwnerInventory)?;
        bytes.extend_from_slice(&route_count.to_be_bytes());
        let count = u32::try_from(self.targets.len()).map_err(|_| FenceError::EmptyOwnerInventory)?;
        bytes.extend_from_slice(&count.to_be_bytes());
        for target in &self.targets {
            append_text(&mut bytes, target.owner.as_str())?;
            bytes.extend_from_slice(&target.generation.to_be_bytes());
        }
        Ok(bytes)
    }
}

/// An effect owner acknowledgement, emitted only after its target generation
/// is durable. The state digest lets external verification bind the exact
/// persisted state; this crate does not authenticate the acknowledgement.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectOwnerFenceAck {
    scope: EffectFenceScope,
    owner: EffectOwnerId,
    installed_generation: u64,
    revocation_digest: Digest,
    durable_state_digest: Digest,
}

impl EffectOwnerFenceAck {
    pub fn new(
        scope: EffectFenceScope,
        owner: EffectOwnerId,
        installed_generation: u64,
        revocation_digest: Digest,
        durable_state_digest: Digest,
    ) -> Result<Self, FenceError> {
        if installed_generation == 0 {
            return Err(FenceError::GenerationNotMonotonic);
        }
        if revocation_digest == [0; 32] || durable_state_digest == [0; 32] {
            return Err(FenceError::ZeroDigest);
        }
        Ok(Self {
            scope,
            owner,
            installed_generation,
            revocation_digest,
            durable_state_digest,
        })
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, FenceError> {
        let mut bytes = b"MYC-DIR-EFFECT-OWNER-FENCE-ACK\0\x01".to_vec();
        self.scope.append_canonical(&mut bytes)?;
        append_text(&mut bytes, self.owner.as_str())?;
        bytes.extend_from_slice(&self.installed_generation.to_be_bytes());
        bytes.extend_from_slice(&self.revocation_digest);
        bytes.extend_from_slice(&self.durable_state_digest);
        Ok(bytes)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RevocationFenceStatus {
    Complete,
    PendingOwnerAcknowledgements,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RevocationFenceRollup {
    status: RevocationFenceStatus,
    acknowledged_owner_count: usize,
    missing_owners: Vec<EffectOwnerId>,
    inventory_epoch: u64,
    route_count: usize,
}

impl RevocationFenceRollup {
    pub fn status(&self) -> RevocationFenceStatus {
        self.status
    }

    pub fn acknowledged_owner_count(&self) -> usize {
        self.acknowledged_owner_count
    }

    pub fn missing_owners(&self) -> &[EffectOwnerId] {
        &self.missing_owners
    }

    pub fn inventory_epoch(&self) -> u64 {
        self.inventory_epoch
    }

    pub fn route_count(&self) -> usize {
        self.route_count
    }
}

/// Requires an exact acknowledgement from every owner in the independently
/// qualified inventory. A quorum is deliberately insufficient for a strict
/// revocation-complete claim.
pub fn assess_revocation_fence(
    plan: &RevocationFencePlan,
    acknowledgements: &[EffectOwnerFenceAck],
) -> Result<RevocationFenceRollup, FenceError> {
    let expected: BTreeMap<EffectOwnerId, u64> = plan
        .targets
        .iter()
        .map(|target| (target.owner.clone(), target.generation))
        .collect();
    let mut seen = BTreeSet::new();

    for acknowledgement in acknowledgements {
        if acknowledgement.scope != plan.scope {
            return Err(FenceError::ScopeMismatch);
        }
        if acknowledgement.revocation_digest != plan.revocation_digest {
            return Err(FenceError::RevocationBindingMismatch);
        }
        let expected_generation = expected
            .get(&acknowledgement.owner)
            .copied()
            .ok_or(FenceError::UnexpectedOwner)?;
        if !seen.insert(acknowledgement.owner.clone()) {
            return Err(FenceError::DuplicateAcknowledgement);
        }
        if acknowledgement.installed_generation != expected_generation {
            return Err(FenceError::GenerationMismatch);
        }
    }

    let missing_owners: Vec<EffectOwnerId> = plan
        .targets
        .iter()
        .filter(|target| !seen.contains(&target.owner))
        .map(|target| target.owner.clone())
        .collect();
    let status = if missing_owners.is_empty() {
        RevocationFenceStatus::Complete
    } else {
        RevocationFenceStatus::PendingOwnerAcknowledgements
    };

    Ok(RevocationFenceRollup {
        status,
        acknowledged_owner_count: seen.len(),
        missing_owners,
        inventory_epoch: plan.inventory_epoch,
        route_count: plan.route_count,
    })
}

/// PEP-signed command binding one exact Allow decision to one effect owner,
/// logical operation, idempotency commitment and current fence token.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FencedEffectCommand {
    request_digest: Digest,
    decision_receipt_digest: Digest,
    logical_operation: LogicalOperationId,
    idempotency_key_digest: Digest,
    token: EffectFenceToken,
}

impl FencedEffectCommand {
    pub fn bind_allow(
        decision: &DecisionReceipt,
        decision_receipt_digest: Digest,
        logical_operation: LogicalOperationId,
        idempotency_key_digest: Digest,
        token: EffectFenceToken,
    ) -> Result<Self, FenceError> {
        if decision.outcome() != DecisionOutcome::Allow {
            return Err(FenceError::RevocationBindingMismatch);
        }
        if decision_receipt_digest == [0; 32] || idempotency_key_digest == [0; 32] {
            return Err(FenceError::ZeroDigest);
        }
        if token.domain.scope.realm() != decision.realm()
        {
            return Err(FenceError::ScopeMismatch);
        }
        Ok(Self {
            request_digest: *decision.request_digest(),
            decision_receipt_digest,
            logical_operation,
            idempotency_key_digest,
            token,
        })
    }

    pub fn token(&self) -> &EffectFenceToken {
        &self.token
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, FenceError> {
        let token = self.token.canonical_bytes()?;
        let token_length = u32::try_from(token.len()).map_err(|_| FenceError::IdentifierTooLong)?;
        let mut bytes = b"MYC-DIR-FENCED-EFFECT-COMMAND\0\x01".to_vec();
        bytes.extend_from_slice(&self.request_digest);
        bytes.extend_from_slice(&self.decision_receipt_digest);
        append_text(&mut bytes, self.logical_operation.as_str())?;
        bytes.extend_from_slice(&self.idempotency_key_digest);
        bytes.extend_from_slice(&token_length.to_be_bytes());
        bytes.extend_from_slice(&token);
        Ok(bytes)
    }

    pub fn assess_at_owner(&self, state: &EffectFenceState) -> FenceAssessment {
        state.assess(&self.token)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::causal_cut::CausalCutAssessment;
    use crate::effect_inventory::{
        EffectRouteEnforcement, EffectRouteId, EffectSurfaceClosureStatus,
        EffectSurfaceKind, EffectSurfaceRoute,
    };
    use crate::receipts::{
        ClockProfileId, PolicyEvaluation, ReceiptId,
    };

    fn scope() -> EffectFenceScope {
        EffectFenceScope::new(
            RealmId::new("realm-a").unwrap(),
            ResourceId::new("payroll-ledger").unwrap(),
            ActionId::new("post-entry").unwrap(),
        )
    }

    fn owner(name: &str) -> EffectOwnerId {
        EffectOwnerId::new(name).unwrap()
    }

    fn decision(evaluation: PolicyEvaluation) -> DecisionReceipt {
        DecisionReceipt::compose(
            ReceiptId::new("decision-1").unwrap(),
            RealmId::new("realm-a").unwrap(),
            [1; 32],
            [2; 32],
            [3; 32],
            [4; 32],
            [5; 32],
            [6; 32],
            [7; 32],
            ClockProfileId::new("monotonic-clock-v1").unwrap(),
            10,
            100,
            CausalCutAssessment::Sufficient,
            evaluation,
        )
        .unwrap()
    }

    fn ack(
        scope: EffectFenceScope,
        name: &str,
        generation: u64,
        revocation: Digest,
    ) -> EffectOwnerFenceAck {
        EffectOwnerFenceAck::new(
            scope,
            owner(name),
            generation,
            revocation,
            [9; 32],
        )
        .unwrap()
    }

    fn two_owner_inventory() -> (EffectSurfaceInventory, EffectSurfaceClosureReceipt) {
        let inventory_scope = scope();
        let revocation = [8; 32];
        let inventory = EffectSurfaceInventory::new(
            inventory_scope.clone(),
            revocation,
            7,
            [2; 32],
            [3; 32],
            [4; 32],
            vec![
                EffectSurfaceRoute::new(
                    EffectRouteId::new("api-write").unwrap(),
                    EffectSurfaceKind::Database,
                    owner("db-a"),
                    EffectRouteEnforcement::AtomicOwnerFence,
                ),
                EffectSurfaceRoute::new(
                    EffectRouteId::new("worker-write").unwrap(),
                    EffectSurfaceKind::MessageBroker,
                    owner("queue-b"),
                    EffectRouteEnforcement::AtomicOwnerFence,
                ),
            ],
        )
        .unwrap();
        let receipt = EffectSurfaceClosureReceipt::new(
            inventory_scope,
            revocation,
            7,
            [2; 32],
            [3; 32],
            [4; 32],
            "effect-inventory-verifier",
            [5; 32],
            [6; 32],
            EffectSurfaceClosureStatus::VerifiedComplete,
        )
        .unwrap();
        (inventory, receipt)
    }

    fn two_owner_plan() -> RevocationFencePlan {
        let (inventory, closure) = two_owner_inventory();
        RevocationFencePlan::from_verified_inventory(
            &inventory,
            &closure,
            vec![
                OwnerFenceTarget::new(owner("db-a"), 4).unwrap(),
                OwnerFenceTarget::new(owner("queue-b"), 9).unwrap(),
            ],
        )
        .unwrap()
    }

    #[test]
    fn current_owner_token_is_accepted() {
        let state = EffectFenceState::new(EffectFenceDomain::new(scope(), owner("db-a")));
        assert_eq!(state.assess(&state.current_token()), FenceAssessment::Current);
    }

    #[test]
    fn previously_current_token_is_stale_after_fence_install() {
        let mut state = EffectFenceState::new(EffectFenceDomain::new(scope(), owner("db-a")));
        let old = state.current_token();
        state.advance_generation().unwrap();
        assert_eq!(state.assess(&old), FenceAssessment::StaleGeneration);
    }

    #[test]
    fn unissued_future_generation_is_rejected() {
        let state = EffectFenceState::new(EffectFenceDomain::new(scope(), owner("db-a")));
        let future = EffectFenceToken::from_wire(state.current_token().domain().clone(), 99);
        assert_eq!(state.assess(&future), FenceAssessment::FutureOrUnissuedGeneration);
    }

    #[test]
    fn token_for_another_resource_owner_domain_is_rejected() {
        let state = EffectFenceState::new(EffectFenceDomain::new(scope(), owner("db-a")));
        let other_scope = EffectFenceScope::new(
            RealmId::new("realm-a").unwrap(),
            ResourceId::new("different-ledger").unwrap(),
            ActionId::new("post-entry").unwrap(),
        );
        let token = EffectFenceToken::from_wire(
            EffectFenceDomain::new(other_scope, owner("db-a")),
            1,
        );
        assert_eq!(state.assess(&token), FenceAssessment::WrongDomain);
    }

    #[test]
    fn generation_install_must_be_strictly_monotonic() {
        let mut state = EffectFenceState::new(EffectFenceDomain::new(scope(), owner("db-a")));
        assert_eq!(state.install_generation(1), Err(FenceError::GenerationNotMonotonic));
        assert_eq!(state.install_generation(0), Err(FenceError::GenerationNotMonotonic));
        assert_eq!(state.install_generation(3).unwrap().generation(), 3);
    }

    #[test]
    fn generation_exhaustion_fails_closed_without_wraparound() {
        let mut state = EffectFenceState::new(EffectFenceDomain::new(scope(), owner("db-a")));
        state.install_generation(u64::MAX).unwrap();
        assert_eq!(state.advance_generation(), Err(FenceError::GenerationExhausted));
        assert_eq!(state.generation(), u64::MAX);
    }

    #[test]
    fn revocation_is_complete_only_after_every_owner_acknowledges() {
        let plan = two_owner_plan();
        let acks = vec![
            ack(scope(), "db-a", 4, [8; 32]),
            ack(scope(), "queue-b", 9, [8; 32]),
        ];
        let result = assess_revocation_fence(&plan, &acks).unwrap();
        assert_eq!(result.status(), RevocationFenceStatus::Complete);
        assert_eq!(result.acknowledged_owner_count(), 2);
        assert!(result.missing_owners().is_empty());
    }

    #[test]
    fn missing_owner_ack_leaves_revocation_pending() {
        let plan = two_owner_plan();
        let result = assess_revocation_fence(
            &plan,
            &[ack(scope(), "db-a", 4, [8; 32])],
        )
        .unwrap();
        assert_eq!(result.status(), RevocationFenceStatus::PendingOwnerAcknowledgements);
        assert_eq!(result.missing_owners(), &[owner("queue-b")]);
    }

    #[test]
    fn duplicate_owner_ack_is_rejected() {
        let plan = two_owner_plan();
        let acks = vec![
            ack(scope(), "db-a", 4, [8; 32]),
            ack(scope(), "db-a", 4, [8; 32]),
        ];
        assert_eq!(
            assess_revocation_fence(&plan, &acks),
            Err(FenceError::DuplicateAcknowledgement)
        );
    }

    #[test]
    fn unknown_owner_ack_is_rejected() {
        let plan = two_owner_plan();
        assert_eq!(
            assess_revocation_fence(&plan, &[ack(scope(), "other", 4, [8; 32])]),
            Err(FenceError::UnexpectedOwner)
        );
    }

    #[test]
    fn wrong_owner_generation_ack_is_rejected() {
        let plan = two_owner_plan();
        assert_eq!(
            assess_revocation_fence(&plan, &[ack(scope(), "db-a", 3, [8; 32])]),
            Err(FenceError::GenerationMismatch)
        );
    }

    #[test]
    fn acknowledgement_must_bind_exact_revocation() {
        let plan = two_owner_plan();
        assert_eq!(
            assess_revocation_fence(&plan, &[ack(scope(), "db-a", 4, [7; 32])]),
            Err(FenceError::RevocationBindingMismatch)
        );
    }

    #[test]
    fn fenced_command_binds_allow_to_owner_generation() {
        let decision = decision(PolicyEvaluation::Permit { witness_digest: [10; 32] });
        assert_eq!(decision.outcome(), crate::receipts::DecisionOutcome::Allow);
        let owner = owner("db-a");
        let state = EffectFenceState::new(EffectFenceDomain::new(scope(), owner));
        let command = FencedEffectCommand::bind_allow(
            &decision,
            [11; 32],
            LogicalOperationId::new("ledger-post-42").unwrap(),
            [12; 32],
            state.current_token(),
        )
        .unwrap();
        assert_eq!(command.assess_at_owner(&state), FenceAssessment::Current);
        assert!(!command.canonical_bytes().unwrap().is_empty());
    }

    #[test]
    fn command_created_before_revocation_is_rejected_after_fence_install() {
        let decision = decision(PolicyEvaluation::Permit { witness_digest: [10; 32] });
        let mut state = EffectFenceState::new(EffectFenceDomain::new(scope(), owner("db-a")));
        let command = FencedEffectCommand::bind_allow(
            &decision,
            [11; 32],
            LogicalOperationId::new("ledger-post-42").unwrap(),
            [12; 32],
            state.current_token(),
        )
        .unwrap();
        state.advance_generation().unwrap();
        assert_eq!(command.assess_at_owner(&state), FenceAssessment::StaleGeneration);
    }

    #[test]
    fn deny_decision_cannot_produce_fenced_effect_command() {
        let decision = decision(PolicyEvaluation::Forbid { forbid_digest: [13; 32] });
        let state = EffectFenceState::new(EffectFenceDomain::new(scope(), owner("db-a")));
        assert_eq!(
            FencedEffectCommand::bind_allow(
                &decision,
                [11; 32],
                LogicalOperationId::new("ledger-post-42").unwrap(),
                [12; 32],
                state.current_token(),
            ),
            Err(FenceError::RevocationBindingMismatch)
        );
    }

    #[test]
    fn owner_targets_must_cover_every_owner_in_verified_route_inventory() {
        let (inventory, closure) = two_owner_inventory();
        assert_eq!(
            RevocationFencePlan::from_verified_inventory(
                &inventory,
                &closure,
                vec![OwnerFenceTarget::new(owner("db-a"), 4).unwrap()],
            ),
            Err(FenceError::EffectOwnerSetMismatch)
        );
    }

    #[test]
    fn unverified_inventory_cannot_produce_strict_revocation_plan() {
        let (inventory, _) = two_owner_inventory();
        let closure = EffectSurfaceClosureReceipt::new(
            scope(),
            [8; 32],
            7,
            [2; 32],
            [3; 32],
            [4; 32],
            "effect-inventory-verifier",
            [5; 32],
            [6; 32],
            EffectSurfaceClosureStatus::Unverified,
        )
        .unwrap();
        assert_eq!(
            RevocationFencePlan::from_verified_inventory(
                &inventory,
                &closure,
                vec![
                    OwnerFenceTarget::new(owner("db-a"), 4).unwrap(),
                    OwnerFenceTarget::new(owner("queue-b"), 9).unwrap(),
                ],
            ),
            Err(FenceError::InventoryNotStronglyCovered)
        );
    }

    #[test]
    fn fence_plan_encoding_is_domain_separated_and_order_independent() {
        let (inventory_a, closure_a) = two_owner_inventory();
        let (inventory_b, closure_b) = two_owner_inventory();
        let plan_a = RevocationFencePlan::from_verified_inventory(
            &inventory_a,
            &closure_a,
            vec![
                OwnerFenceTarget::new(owner("db-a"), 4).unwrap(),
                OwnerFenceTarget::new(owner("queue-b"), 9).unwrap(),
            ],
        )
        .unwrap();
        let plan_b = RevocationFencePlan::from_verified_inventory(
            &inventory_b,
            &closure_b,
            vec![
                OwnerFenceTarget::new(owner("queue-b"), 9).unwrap(),
                OwnerFenceTarget::new(owner("db-a"), 4).unwrap(),
            ],
        )
        .unwrap();
        assert_eq!(plan_a.canonical_bytes().unwrap(), plan_b.canonical_bytes().unwrap());
    }
}
