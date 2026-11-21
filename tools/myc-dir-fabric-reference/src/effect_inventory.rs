//! Complete effect-surface inventory contracts for strict revocation.
//!
//! Owner acknowledgements are meaningful only if the inventory enumerating all
//! mutation routes is complete. This module binds one revocation and exact
//! resource scope to a versioned route inventory and an external completeness
//! proof. It does not discover routes, hash, sign, or verify proofs itself.

use crate::effect_fence::EffectFenceScope;
use crate::receipts::EffectOwnerId;
use crate::Digest;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum InventoryError {
    EmptyIdentifier,
    IdentifierTooLong,
    ZeroDigest,
    InvalidInventoryEpoch,
    EmptyRouteInventory,
    DuplicateRouteId,
}

impl std::fmt::Display for InventoryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for InventoryError {}

fn checked_text(value: impl Into<String>) -> Result<String, InventoryError> {
    let value = value.into();
    if value.trim().is_empty() {
        return Err(InventoryError::EmptyIdentifier);
    }
    if value.len() > 512 {
        return Err(InventoryError::IdentifierTooLong);
    }
    Ok(value)
}

fn append_text(bytes: &mut Vec<u8>, value: &str) -> Result<(), InventoryError> {
    let value = checked_text(value.to_owned())?;
    let length = u16::try_from(value.len()).map_err(|_| InventoryError::IdentifierTooLong)?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(value.as_bytes());
    Ok(())
}

fn require_digest(digest: &Digest) -> Result<(), InventoryError> {
    if digest == &[0; 32] {
        return Err(InventoryError::ZeroDigest);
    }
    Ok(())
}

/// A distinct ingress or mutation route that can reach an effect surface.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct EffectRouteId(String);

impl EffectRouteId {
    pub fn new(value: impl Into<String>) -> Result<Self, InventoryError> {
        checked_text(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Route classes are descriptive inventory facts, not permission types.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
#[repr(u8)]
pub enum EffectSurfaceKind {
    ApiGateway = 1,
    SessionService = 2,
    TokenService = 3,
    Worker = 4,
    MessageBroker = 5,
    Database = 6,
    DeviceAgent = 7,
    Filesystem = 8,
    ExternalApi = 9,
    Other = 10,
}

/// Only AtomicOwnerFence can count toward a strict mutation-revocation claim.
/// CAEP, token expiry and best-effort invalidation are attenuation mechanisms,
/// but are not atomic mutation-boundary fencing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum EffectRouteEnforcement {
    AtomicOwnerFence = 1,
    CaepSignalOnly = 2,
    TokenExpiryOnly = 3,
    BestEffortInvalidation = 4,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectSurfaceRoute {
    route_id: EffectRouteId,
    kind: EffectSurfaceKind,
    owner: EffectOwnerId,
    enforcement: EffectRouteEnforcement,
}

impl EffectSurfaceRoute {
    pub fn new(
        route_id: EffectRouteId,
        kind: EffectSurfaceKind,
        owner: EffectOwnerId,
        enforcement: EffectRouteEnforcement,
    ) -> Self {
        Self {
            route_id,
            kind,
            owner,
            enforcement,
        }
    }

    pub fn route_id(&self) -> &EffectRouteId {
        &self.route_id
    }

    pub fn kind(&self) -> EffectSurfaceKind {
        self.kind
    }

    pub fn owner(&self) -> &EffectOwnerId {
        &self.owner
    }

    pub fn enforcement(&self) -> EffectRouteEnforcement {
        self.enforcement
    }
}

/// One revocation-specific, versioned inventory of every route that can carry
/// the revoked authority to a protected mutation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectSurfaceInventory {
    scope: EffectFenceScope,
    revocation_digest: Digest,
    inventory_epoch: u64,
    topology_snapshot_digest: Digest,
    inventory_manifest_digest: Digest,
    route_set_digest: Digest,
    routes: Vec<EffectSurfaceRoute>,
}

impl EffectSurfaceInventory {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        scope: EffectFenceScope,
        revocation_digest: Digest,
        inventory_epoch: u64,
        topology_snapshot_digest: Digest,
        inventory_manifest_digest: Digest,
        route_set_digest: Digest,
        mut routes: Vec<EffectSurfaceRoute>,
    ) -> Result<Self, InventoryError> {
        if inventory_epoch == 0 {
            return Err(InventoryError::InvalidInventoryEpoch);
        }
        for digest in [
            &revocation_digest,
            &topology_snapshot_digest,
            &inventory_manifest_digest,
            &route_set_digest,
        ] {
            require_digest(digest)?;
        }
        if routes.is_empty() {
            return Err(InventoryError::EmptyRouteInventory);
        }
        routes.sort_by(|left, right| left.route_id.cmp(&right.route_id));
        for pair in routes.windows(2) {
            if pair[0].route_id == pair[1].route_id {
                return Err(InventoryError::DuplicateRouteId);
            }
        }
        Ok(Self {
            scope,
            revocation_digest,
            inventory_epoch,
            topology_snapshot_digest,
            inventory_manifest_digest,
            route_set_digest,
            routes,
        })
    }

    pub fn scope(&self) -> &EffectFenceScope {
        &self.scope
    }

    pub fn revocation_digest(&self) -> &Digest {
        &self.revocation_digest
    }

    pub fn inventory_epoch(&self) -> u64 {
        self.inventory_epoch
    }

    pub fn inventory_manifest_digest(&self) -> &Digest {
        &self.inventory_manifest_digest
    }

    pub fn route_set_digest(&self) -> &Digest {
        &self.route_set_digest
    }

    pub fn routes(&self) -> &[EffectSurfaceRoute] {
        &self.routes
    }

    /// Canonical material whose digest is the inventory manifest digest.
    /// The manifest digest itself is excluded to avoid self-reference.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, InventoryError> {
        let mut bytes = b"MYC-DIR-EFFECT-SURFACE-INVENTORY\0\x01".to_vec();
        append_text(&mut bytes, self.scope.realm().as_str())?;
        append_text(&mut bytes, self.scope.resource().as_str())?;
        append_text(&mut bytes, self.scope.action().as_str())?;
        bytes.extend_from_slice(&self.revocation_digest);
        bytes.extend_from_slice(&self.inventory_epoch.to_be_bytes());
        bytes.extend_from_slice(&self.topology_snapshot_digest);
        bytes.extend_from_slice(&self.route_set_digest);
        let route_count =
            u32::try_from(self.routes.len()).map_err(|_| InventoryError::EmptyRouteInventory)?;
        bytes.extend_from_slice(&route_count.to_be_bytes());
        for route in &self.routes {
            append_text(&mut bytes, route.route_id.as_str())?;
            bytes.push(route.kind as u8);
            append_text(&mut bytes, route.owner.as_str())?;
            bytes.push(route.enforcement as u8);
        }
        Ok(bytes)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum EffectSurfaceClosureStatus {
    VerifiedComplete = 1,
    Incomplete = 2,
    Rejected = 3,
    Unavailable = 4,
    Unverified = 5,
}

/// External verifier's claim about one exact effect-surface inventory.
/// This crate validates structural binding only; it cannot authenticate the
/// verifier or establish that its route-discovery procedure is trustworthy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectSurfaceClosureReceipt {
    scope: EffectFenceScope,
    revocation_digest: Digest,
    inventory_epoch: u64,
    topology_snapshot_digest: Digest,
    inventory_manifest_digest: Digest,
    route_set_digest: Digest,
    verifier_id: String,
    verifier_build_digest: Digest,
    proof_digest: Digest,
    status: EffectSurfaceClosureStatus,
}

impl EffectSurfaceClosureReceipt {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        scope: EffectFenceScope,
        revocation_digest: Digest,
        inventory_epoch: u64,
        topology_snapshot_digest: Digest,
        inventory_manifest_digest: Digest,
        route_set_digest: Digest,
        verifier_id: impl Into<String>,
        verifier_build_digest: Digest,
        proof_digest: Digest,
        status: EffectSurfaceClosureStatus,
    ) -> Result<Self, InventoryError> {
        if inventory_epoch == 0 {
            return Err(InventoryError::InvalidInventoryEpoch);
        }
        for digest in [
            &revocation_digest,
            &topology_snapshot_digest,
            &inventory_manifest_digest,
            &route_set_digest,
            &verifier_build_digest,
            &proof_digest,
        ] {
            require_digest(digest)?;
        }
        Ok(Self {
            scope,
            revocation_digest,
            inventory_epoch,
            topology_snapshot_digest,
            inventory_manifest_digest,
            route_set_digest,
            verifier_id: checked_text(verifier_id)?,
            verifier_build_digest,
            proof_digest,
            status,
        })
    }

    pub fn status(&self) -> EffectSurfaceClosureStatus {
        self.status
    }

    pub fn proof_digest(&self) -> &Digest {
        &self.proof_digest
    }

    pub fn verifier_id(&self) -> &str {
        &self.verifier_id
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, InventoryError> {
        let mut bytes = b"MYC-DIR-EFFECT-SURFACE-CLOSURE-RECEIPT\0\x01".to_vec();
        append_text(&mut bytes, self.scope.realm().as_str())?;
        append_text(&mut bytes, self.scope.resource().as_str())?;
        append_text(&mut bytes, self.scope.action().as_str())?;
        bytes.extend_from_slice(&self.revocation_digest);
        bytes.extend_from_slice(&self.inventory_epoch.to_be_bytes());
        bytes.extend_from_slice(&self.topology_snapshot_digest);
        bytes.extend_from_slice(&self.inventory_manifest_digest);
        bytes.extend_from_slice(&self.route_set_digest);
        append_text(&mut bytes, &self.verifier_id)?;
        bytes.extend_from_slice(&self.verifier_build_digest);
        bytes.extend_from_slice(&self.proof_digest);
        bytes.push(self.status as u8);
        Ok(bytes)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EffectSurfaceAssessment {
    StrongFenceEligible,
    BestEffortOnly,
    ClosureUnverified,
    ClosureIncomplete,
    ClosureRejected,
    ClosureUnavailable,
    BindingMismatch,
}

/// StrongFenceEligible means only that an external VerifiedComplete claim is
/// structurally bound and every enumerated route advertises atomic owner
/// fencing. It is not proof of verifier authenticity, inventory truth,
/// signature validity or operational reachability.
pub fn assess_effect_surface_closure(
    inventory: &EffectSurfaceInventory,
    receipt: &EffectSurfaceClosureReceipt,
) -> EffectSurfaceAssessment {
    if receipt.scope != inventory.scope
        || receipt.revocation_digest != inventory.revocation_digest
        || receipt.inventory_epoch != inventory.inventory_epoch
        || receipt.topology_snapshot_digest != inventory.topology_snapshot_digest
        || receipt.inventory_manifest_digest != inventory.inventory_manifest_digest
        || receipt.route_set_digest != inventory.route_set_digest
    {
        return EffectSurfaceAssessment::BindingMismatch;
    }

    match receipt.status {
        EffectSurfaceClosureStatus::VerifiedComplete => {
            if inventory.routes.iter().all(|route| {
                route.enforcement == EffectRouteEnforcement::AtomicOwnerFence
            }) {
                EffectSurfaceAssessment::StrongFenceEligible
            } else {
                EffectSurfaceAssessment::BestEffortOnly
            }
        }
        EffectSurfaceClosureStatus::Incomplete => EffectSurfaceAssessment::ClosureIncomplete,
        EffectSurfaceClosureStatus::Rejected => EffectSurfaceAssessment::ClosureRejected,
        EffectSurfaceClosureStatus::Unavailable => EffectSurfaceAssessment::ClosureUnavailable,
        EffectSurfaceClosureStatus::Unverified => EffectSurfaceAssessment::ClosureUnverified,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::effect_fence::EffectFenceScope;
    use crate::receipts::EffectOwnerId;
    use crate::{ActionId, RealmId, ResourceId};

    fn scope() -> EffectFenceScope {
        EffectFenceScope::new(
            RealmId::new("realm-a").unwrap(),
            ResourceId::new("payroll-ledger").unwrap(),
            ActionId::new("post-entry").unwrap(),
        )
    }

    fn route(id: &str, owner: &str, enforcement: EffectRouteEnforcement) -> EffectSurfaceRoute {
        EffectSurfaceRoute::new(
            EffectRouteId::new(id).unwrap(),
            EffectSurfaceKind::Database,
            EffectOwnerId::new(owner).unwrap(),
            enforcement,
        )
    }

    fn inventory(routes: Vec<EffectSurfaceRoute>) -> EffectSurfaceInventory {
        EffectSurfaceInventory::new(
            scope(), [1; 32], 7, [2; 32], [3; 32], [4; 32], routes,
        )
        .unwrap()
    }

    fn receipt(
        inventory: &EffectSurfaceInventory,
        status: EffectSurfaceClosureStatus,
    ) -> EffectSurfaceClosureReceipt {
        EffectSurfaceClosureReceipt::new(
            inventory.scope.clone(),
            *inventory.revocation_digest(),
            inventory.inventory_epoch(),
            [2; 32],
            [3; 32],
            [4; 32],
            "closure-verifier-1",
            [5; 32],
            [6; 32],
            status,
        )
        .unwrap()
    }

    #[test]
    fn complete_inventory_with_atomic_fences_is_strong_eligible() {
        let inventory = inventory(vec![
            route("gateway-write", "db-a", EffectRouteEnforcement::AtomicOwnerFence),
            route("worker-write", "queue-b", EffectRouteEnforcement::AtomicOwnerFence),
        ]);
        assert_eq!(
            assess_effect_surface_closure(
                &inventory,
                &receipt(&inventory, EffectSurfaceClosureStatus::VerifiedComplete)
            ),
            EffectSurfaceAssessment::StrongFenceEligible
        );
    }

    #[test]
    fn caep_signal_only_route_never_counts_as_strong_mutation_fence() {
        let inventory = inventory(vec![
            route("session", "session-a", EffectRouteEnforcement::AtomicOwnerFence),
            route("legacy-api", "legacy-b", EffectRouteEnforcement::CaepSignalOnly),
        ]);
        assert_eq!(
            assess_effect_surface_closure(
                &inventory,
                &receipt(&inventory, EffectSurfaceClosureStatus::VerifiedComplete)
            ),
            EffectSurfaceAssessment::BestEffortOnly
        );
    }

    #[test]
    fn token_expiry_only_route_is_best_effort_not_strict() {
        let inventory = inventory(vec![
            route("jwt-resource-server", "api-a", EffectRouteEnforcement::TokenExpiryOnly),
        ]);
        assert_eq!(
            assess_effect_surface_closure(
                &inventory,
                &receipt(&inventory, EffectSurfaceClosureStatus::VerifiedComplete)
            ),
            EffectSurfaceAssessment::BestEffortOnly
        );
    }

    #[test]
    fn incomplete_route_closure_never_eligible() {
        let inventory = inventory(vec![
            route("db", "db-a", EffectRouteEnforcement::AtomicOwnerFence),
        ]);
        assert_eq!(
            assess_effect_surface_closure(
                &inventory,
                &receipt(&inventory, EffectSurfaceClosureStatus::Incomplete)
            ),
            EffectSurfaceAssessment::ClosureIncomplete
        );
    }

    #[test]
    fn unverified_route_closure_never_eligible() {
        let inventory = inventory(vec![
            route("db", "db-a", EffectRouteEnforcement::AtomicOwnerFence),
        ]);
        assert_eq!(
            assess_effect_surface_closure(
                &inventory,
                &receipt(&inventory, EffectSurfaceClosureStatus::Unverified)
            ),
            EffectSurfaceAssessment::ClosureUnverified
        );
    }

    #[test]
    fn graph_snapshot_substitution_is_rejected() {
        let inventory = inventory(vec![
            route("db", "db-a", EffectRouteEnforcement::AtomicOwnerFence),
        ]);
        let wrong = EffectSurfaceClosureReceipt::new(
            inventory.scope.clone(), [1; 32], 7, [99; 32], [3; 32], [4; 32],
            "closure-verifier-1", [5; 32], [6; 32],
            EffectSurfaceClosureStatus::VerifiedComplete,
        ).unwrap();
        assert_eq!(
            assess_effect_surface_closure(&inventory, &wrong),
            EffectSurfaceAssessment::BindingMismatch
        );
    }

    #[test]
    fn route_manifest_substitution_is_rejected() {
        let inventory = inventory(vec![
            route("db", "db-a", EffectRouteEnforcement::AtomicOwnerFence),
        ]);
        let wrong = EffectSurfaceClosureReceipt::new(
            inventory.scope.clone(), [1; 32], 7, [2; 32], [3; 32], [99; 32],
            "closure-verifier-1", [5; 32], [6; 32],
            EffectSurfaceClosureStatus::VerifiedComplete,
        ).unwrap();
        assert_eq!(
            assess_effect_surface_closure(&inventory, &wrong),
            EffectSurfaceAssessment::BindingMismatch
        );
    }

    #[test]
    fn duplicate_route_ids_are_rejected() {
        assert_eq!(
            EffectSurfaceInventory::new(
                scope(), [1; 32], 7, [2; 32], [3; 32], [4; 32],
                vec![
                    route("same", "db-a", EffectRouteEnforcement::AtomicOwnerFence),
                    route("same", "queue-b", EffectRouteEnforcement::AtomicOwnerFence),
                ],
            ),
            Err(InventoryError::DuplicateRouteId)
        );
    }

    #[test]
    fn empty_inventory_is_rejected() {
        assert_eq!(
            EffectSurfaceInventory::new(
                scope(), [1; 32], 7, [2; 32], [3; 32], [4; 32], vec![],
            ),
            Err(InventoryError::EmptyRouteInventory)
        );
    }

    #[test]
    fn inventory_encoding_is_order_independent() {
        let first = inventory(vec![
            route("db-a", "owner-a", EffectRouteEnforcement::AtomicOwnerFence),
            route("queue-b", "owner-b", EffectRouteEnforcement::AtomicOwnerFence),
        ]);
        let second = inventory(vec![
            route("queue-b", "owner-b", EffectRouteEnforcement::AtomicOwnerFence),
            route("db-a", "owner-a", EffectRouteEnforcement::AtomicOwnerFence),
        ]);
        assert_eq!(first.canonical_bytes().unwrap(), second.canonical_bytes().unwrap());
    }
}
