//! Owner-epoch recovery, post-revocation route admission and barrier receipts.
//!
//! This module extends the generation-only reference contract with two
//! boundaries that must survive real deployments:
//! - an owner restore/failover cannot reopen writes below its acknowledged
//!   owner-epoch/generation high-water mark;
//! - a newly introduced effect route cannot become executable until it is
//!   bound to the current owner fence.
//!
//! All digests are externally computed commitments. This module does not hash,
//! sign, authenticate, verify a trust root, or make storage durable. Production
//! owners must atomically enforce the returned position with the mutation.

use std::collections::{BTreeMap, BTreeSet};

use crate::effect_fence::EffectFenceScope;
use crate::effect_inventory::{EffectRouteEnforcement, EffectRouteId};
use crate::receipts::EffectOwnerId;
use crate::{Digest, MAX_IDENTIFIER_BYTES};

fn checked_text(value: &str) -> Result<(), OwnerEpochError> {
    if value.trim().is_empty() {
        return Err(OwnerEpochError::EmptyIdentifier);
    }
    if value.len() > MAX_IDENTIFIER_BYTES {
        return Err(OwnerEpochError::IdentifierTooLong);
    }
    Ok(())
}

fn append_text(bytes: &mut Vec<u8>, value: &str) -> Result<(), OwnerEpochError> {
    checked_text(value)?;
    let length = u16::try_from(value.len()).map_err(|_| OwnerEpochError::IdentifierTooLong)?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(value.as_bytes());
    Ok(())
}

fn require_digest(digest: &Digest) -> Result<(), OwnerEpochError> {
    if digest == &[0; 32] {
        return Err(OwnerEpochError::ZeroDigest);
    }
    Ok(())
}

fn append_position(bytes: &mut Vec<u8>, position: OwnerFencePosition) {
    bytes.extend_from_slice(&position.owner_epoch.to_be_bytes());
    bytes.extend_from_slice(&position.generation.to_be_bytes());
}

/// The fencing coordinate is lexicographic: owner epoch first, then the
/// resource/action generation within that epoch. A recovered owner must use a
/// new epoch and may not lower the acknowledged generation high-water mark.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct OwnerFencePosition {
    owner_epoch: u64,
    generation: u64,
}

impl OwnerFencePosition {
    pub fn new(owner_epoch: u64, generation: u64) -> Result<Self, OwnerEpochError> {
        if owner_epoch == 0 || generation == 0 {
            return Err(OwnerEpochError::ZeroPosition);
        }
        Ok(Self {
            owner_epoch,
            generation,
        })
    }

    pub fn owner_epoch(self) -> u64 {
        self.owner_epoch
    }

    pub fn generation(self) -> u64 {
        self.generation
    }

    pub fn canonical_bytes(self) -> Vec<u8> {
        let mut bytes = b"MYC-DIR-OWNER-FENCE-POSITION\0\x01".to_vec();
        append_position(&mut bytes, self);
        bytes
    }
}

/// Externally durable high-water evidence. The receipt digest must be
/// independently authenticated before it is supplied to this reference layer.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerHighWaterMark {
    owner: EffectOwnerId,
    position: OwnerFencePosition,
    receipt_digest: Digest,
}

impl OwnerHighWaterMark {
    pub fn new(
        owner: EffectOwnerId,
        position: OwnerFencePosition,
        receipt_digest: Digest,
    ) -> Result<Self, OwnerEpochError> {
        require_digest(&receipt_digest)?;
        Ok(Self {
            owner,
            position,
            receipt_digest,
        })
    }

    pub fn owner(&self) -> &EffectOwnerId {
        &self.owner
    }

    pub fn position(&self) -> OwnerFencePosition {
        self.position
    }

    pub fn receipt_digest(&self) -> &Digest {
        &self.receipt_digest
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, OwnerEpochError> {
        let mut bytes = b"MYC-DIR-OWNER-HIGH-WATER\0\x01".to_vec();
        append_text(&mut bytes, self.owner.as_str())?;
        append_position(&mut bytes, self.position);
        bytes.extend_from_slice(&self.receipt_digest);
        Ok(bytes)
    }
}

/// Runtime view of one effect owner. Unverified restore candidates are always
/// closed to mutations until recovery evidence and a durable read-back are
/// supplied. The high-water mark is modeled separately from the restored
/// candidate because a stale backup must not overwrite the last acknowledgement.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerEpochState {
    owner: EffectOwnerId,
    current: OwnerFencePosition,
    high_water: OwnerHighWaterMark,
    recovery_ready: bool,
    durable_position: Option<OwnerFencePosition>,
    owner_commit_digest: Option<Digest>,
    durable_readback_digest: Option<Digest>,
}

impl OwnerEpochState {
    /// Constructs the initial state only when bootstrap evidence and a durable
    /// read-back commitment are already available from the owner control plane.
    pub fn from_verified_bootstrap(
        owner: EffectOwnerId,
        position: OwnerFencePosition,
        bootstrap_receipt_digest: Digest,
        durable_readback_digest: Digest,
    ) -> Result<Self, OwnerEpochError> {
        require_digest(&bootstrap_receipt_digest)?;
        require_digest(&durable_readback_digest)?;
        let high_water =
            OwnerHighWaterMark::new(owner.clone(), position, bootstrap_receipt_digest)?;
        Ok(Self {
            owner,
            current: position,
            high_water,
            recovery_ready: true,
            durable_position: Some(position),
            owner_commit_digest: Some(bootstrap_receipt_digest),
            durable_readback_digest: Some(durable_readback_digest),
        })
    }

    /// A restore candidate is deliberately not writable, even when its
    /// observed position appears current. It must complete an explicit
    /// high-water reconciliation and durable read-back first.
    pub fn from_unverified_restore(
        owner: EffectOwnerId,
        restored_candidate: OwnerFencePosition,
        high_water: OwnerHighWaterMark,
    ) -> Result<Self, OwnerEpochError> {
        if high_water.owner != owner {
            return Err(OwnerEpochError::OwnerMismatch);
        }
        Ok(Self {
            owner,
            current: restored_candidate,
            high_water,
            recovery_ready: false,
            durable_position: None,
            owner_commit_digest: None,
            durable_readback_digest: None,
        })
    }

    pub fn owner(&self) -> &EffectOwnerId {
        &self.owner
    }

    pub fn current_position(&self) -> OwnerFencePosition {
        self.current
    }

    pub fn high_water(&self) -> &OwnerHighWaterMark {
        &self.high_water
    }

    pub fn recovery_ready(&self) -> bool {
        self.recovery_ready
    }

    pub fn current_is_durable(&self) -> bool {
        self.durable_position == Some(self.current)
            && self.owner_commit_digest.is_some()
            && self.durable_readback_digest.is_some()
    }

    /// Record the exact durable commit and post-commit read-back for the
    /// current position. A route or owner acknowledgement cannot rely on a
    /// merely proposed/in-memory generation.
    pub fn record_durable_commit_readback(
        &mut self,
        position: OwnerFencePosition,
        owner_commit_digest: Digest,
        durable_readback_digest: Digest,
    ) -> Result<(), OwnerEpochError> {
        if !self.recovery_ready {
            return Err(OwnerEpochError::RecoveryRequired);
        }
        if position != self.current {
            return Err(OwnerEpochError::AcknowledgementPositionMismatch);
        }
        require_digest(&owner_commit_digest)?;
        require_digest(&durable_readback_digest)?;
        self.durable_position = Some(position);
        self.owner_commit_digest = Some(owner_commit_digest);
        self.durable_readback_digest = Some(durable_readback_digest);
        Ok(())
    }

    /// Reconcile a restored or failed-over owner to a fresh owner epoch. The
    /// new epoch must exceed both the candidate and durable high-water epochs;
    /// the generation must not regress below either observed position. The
    /// receipt and read-back digests are commitments supplied by independent,
    /// deployment-specific verification.
    pub fn reconcile_recovery(
        &mut self,
        new_owner_epoch: u64,
        recovered_generation: u64,
        recovery_receipt_digest: Digest,
        durable_readback_digest: Digest,
    ) -> Result<OwnerFencePosition, OwnerEpochError> {
        require_digest(&recovery_receipt_digest)?;
        require_digest(&durable_readback_digest)?;
        let minimum_epoch = self.current.owner_epoch.max(self.high_water.position.owner_epoch);
        if new_owner_epoch <= minimum_epoch {
            return Err(OwnerEpochError::OwnerEpochNotAdvanced);
        }
        let minimum_generation = self
            .current
            .generation
            .max(self.high_water.position.generation);
        if recovered_generation < minimum_generation {
            return Err(OwnerEpochError::GenerationBelowHighWater);
        }

        let recovered = OwnerFencePosition::new(new_owner_epoch, recovered_generation)?;
        self.current = recovered;
        self.high_water =
            OwnerHighWaterMark::new(self.owner.clone(), recovered, recovery_receipt_digest)?;
        self.recovery_ready = true;
        self.durable_position = Some(recovered);
        self.owner_commit_digest = Some(recovery_receipt_digest);
        self.durable_readback_digest = Some(durable_readback_digest);
        Ok(recovered)
    }

    /// Advances the generation inside the current owner epoch. This reference
    /// transition is not a durable acknowledgement; the caller must commit,
    /// read back and then record the owner acknowledgement.
    pub fn install_next_generation(&mut self) -> Result<OwnerFencePosition, OwnerEpochError> {
        if !self.recovery_ready {
            return Err(OwnerEpochError::RecoveryRequired);
        }
        let next = self
            .current
            .generation
            .checked_add(1)
            .ok_or(OwnerEpochError::GenerationExhausted)?;
        self.current = OwnerFencePosition::new(self.current.owner_epoch, next)?;
        self.durable_position = None;
        self.owner_commit_digest = None;
        self.durable_readback_digest = None;
        Ok(self.current)
    }

    /// Advances the externally durable high-water only for an exact
    /// acknowledgement that matches the current owner state and prior mark.
    /// The supplied digest is the external hash of acknowledgement canonical
    /// bytes; signature verification and durable storage are outside this crate.
    pub fn record_acknowledgement(
        &mut self,
        acknowledgement: &OwnerEpochFenceAck,
        acknowledgement_digest: Digest,
        expected_scope: &EffectFenceScope,
        expected_revocation_digest: Digest,
        expected_inventory_epoch: u64,
        expected_inventory_manifest_digest: Digest,
        expected_route_set_digest: Digest,
    ) -> Result<(), OwnerEpochError> {
        require_digest(&acknowledgement_digest)?;
        if !self.recovery_ready {
            return Err(OwnerEpochError::RecoveryRequired);
        }
        if acknowledgement.owner != self.owner {
            return Err(OwnerEpochError::OwnerMismatch);
        }
        if acknowledgement.scope != *expected_scope {
            return Err(OwnerEpochError::ScopeMismatch);
        }
        if acknowledgement.revocation_digest != expected_revocation_digest {
            return Err(OwnerEpochError::RevocationBindingMismatch);
        }
        if acknowledgement.inventory_epoch != expected_inventory_epoch {
            return Err(OwnerEpochError::InventoryEpochMismatch);
        }
        if acknowledgement.inventory_manifest_digest != expected_inventory_manifest_digest {
            return Err(OwnerEpochError::InventoryManifestMismatch);
        }
        if acknowledgement.route_set_digest != expected_route_set_digest {
            return Err(OwnerEpochError::RouteSetMismatch);
        }
        if acknowledgement.previous_position != self.high_water.position
            || acknowledgement.installed_position != self.current
            || acknowledgement.installed_position <= acknowledgement.previous_position
        {
            return Err(OwnerEpochError::AcknowledgementPositionMismatch);
        }
        if !self.current_is_durable() {
            return Err(OwnerEpochError::OwnerFenceNotDurablyReadBack);
        }
        if acknowledgement.owner_commit_digest != self.owner_commit_digest.unwrap()
            || acknowledgement.durable_readback_digest != self.durable_readback_digest.unwrap()
        {
            return Err(OwnerEpochError::AcknowledgementEvidenceMismatch);
        }
        self.high_water = OwnerHighWaterMark::new(
            self.owner.clone(),
            self.current,
            acknowledgement_digest,
        )?;
        Ok(())
    }

    /// A command is current only when both owner epoch and generation match.
    /// A matching generation in an old or restored owner epoch is rejected.
    pub fn assess_position(&self, supplied: OwnerFencePosition) -> OwnerPositionAssessment {
        if !self.recovery_ready {
            return OwnerPositionAssessment::RecoveryRequired;
        }
        if supplied.owner_epoch != self.current.owner_epoch {
            return OwnerPositionAssessment::StaleOwnerEpoch;
        }
        match supplied.generation.cmp(&self.current.generation) {
            std::cmp::Ordering::Less => OwnerPositionAssessment::StaleGeneration,
            std::cmp::Ordering::Equal => OwnerPositionAssessment::Current,
            std::cmp::Ordering::Greater => OwnerPositionAssessment::FutureOrUnissuedGeneration,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OwnerPositionAssessment {
    Current,
    StaleOwnerEpoch,
    StaleGeneration,
    FutureOrUnissuedGeneration,
    RecoveryRequired,
}

/// Owner acknowledgement binding the previous and installed fencing positions
/// to one revocation and one exact inventory closure.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerEpochFenceAck {
    scope: EffectFenceScope,
    owner: EffectOwnerId,
    previous_position: OwnerFencePosition,
    installed_position: OwnerFencePosition,
    revocation_digest: Digest,
    inventory_epoch: u64,
    inventory_manifest_digest: Digest,
    route_set_digest: Digest,
    owner_commit_digest: Digest,
    durable_readback_digest: Digest,
}

impl OwnerEpochFenceAck {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        scope: EffectFenceScope,
        owner: EffectOwnerId,
        previous_position: OwnerFencePosition,
        installed_position: OwnerFencePosition,
        revocation_digest: Digest,
        inventory_epoch: u64,
        inventory_manifest_digest: Digest,
        route_set_digest: Digest,
        owner_commit_digest: Digest,
        durable_readback_digest: Digest,
    ) -> Result<Self, OwnerEpochError> {
        if installed_position <= previous_position {
            return Err(OwnerEpochError::AcknowledgementPositionNotMonotonic);
        }
        if inventory_epoch == 0 {
            return Err(OwnerEpochError::InvalidInventoryEpoch);
        }
        for digest in [
            &revocation_digest,
            &inventory_manifest_digest,
            &route_set_digest,
            &owner_commit_digest,
            &durable_readback_digest,
        ] {
            require_digest(digest)?;
        }
        Ok(Self {
            scope,
            owner,
            previous_position,
            installed_position,
            revocation_digest,
            inventory_epoch,
            inventory_manifest_digest,
            route_set_digest,
            owner_commit_digest,
            durable_readback_digest,
        })
    }

    pub fn owner(&self) -> &EffectOwnerId {
        &self.owner
    }

    pub fn previous_position(&self) -> OwnerFencePosition {
        self.previous_position
    }

    pub fn installed_position(&self) -> OwnerFencePosition {
        self.installed_position
    }

    pub fn inventory_epoch(&self) -> u64 {
        self.inventory_epoch
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, OwnerEpochError> {
        let mut bytes = b"MYC-DIR-OWNER-EPOCH-FENCE-ACK\0\x01".to_vec();
        append_text(&mut bytes, self.scope.realm().as_str())?;
        append_text(&mut bytes, self.scope.resource().as_str())?;
        append_text(&mut bytes, self.scope.action().as_str())?;
        append_text(&mut bytes, self.owner.as_str())?;
        append_position(&mut bytes, self.previous_position);
        append_position(&mut bytes, self.installed_position);
        bytes.extend_from_slice(&self.revocation_digest);
        bytes.extend_from_slice(&self.inventory_epoch.to_be_bytes());
        bytes.extend_from_slice(&self.inventory_manifest_digest);
        bytes.extend_from_slice(&self.route_set_digest);
        bytes.extend_from_slice(&self.owner_commit_digest);
        bytes.extend_from_slice(&self.durable_readback_digest);
        Ok(bytes)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerEpochFenceTarget {
    owner: EffectOwnerId,
    previous_position: OwnerFencePosition,
    installed_position: OwnerFencePosition,
}

impl OwnerEpochFenceTarget {
    pub fn new(
        owner: EffectOwnerId,
        previous_position: OwnerFencePosition,
        installed_position: OwnerFencePosition,
    ) -> Result<Self, OwnerEpochError> {
        if installed_position <= previous_position {
            return Err(OwnerEpochError::AcknowledgementPositionNotMonotonic);
        }
        Ok(Self {
            owner,
            previous_position,
            installed_position,
        })
    }

    pub fn owner(&self) -> &EffectOwnerId {
        &self.owner
    }

    pub fn installed_position(&self) -> OwnerFencePosition {
        self.installed_position
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OwnerEpochBarrierStatus {
    Complete,
    PendingOwnerAcknowledgements,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerEpochFenceRollup {
    scope: EffectFenceScope,
    revocation_digest: Digest,
    inventory_epoch: u64,
    inventory_manifest_digest: Digest,
    route_set_digest: Digest,
    status: OwnerEpochBarrierStatus,
    acknowledged_owner_count: usize,
    expected_owner_count: usize,
    missing_owners: Vec<EffectOwnerId>,
}

impl OwnerEpochFenceRollup {
    pub fn status(&self) -> OwnerEpochBarrierStatus {
        self.status
    }

    pub fn acknowledged_owner_count(&self) -> usize {
        self.acknowledged_owner_count
    }

    pub fn expected_owner_count(&self) -> usize {
        self.expected_owner_count
    }

    pub fn missing_owners(&self) -> &[EffectOwnerId] {
        &self.missing_owners
    }
}

/// Independently computed from canonical_owner_epoch_ack_set_bytes(). No
/// quorum shortcut is accepted: every expected owner must have one exact ack.
pub fn assess_owner_epoch_fence_acks(
    scope: &EffectFenceScope,
    revocation_digest: Digest,
    inventory_epoch: u64,
    inventory_manifest_digest: Digest,
    route_set_digest: Digest,
    targets: &[OwnerEpochFenceTarget],
    acknowledgements: &[OwnerEpochFenceAck],
) -> Result<OwnerEpochFenceRollup, OwnerEpochError> {
    if inventory_epoch == 0 {
        return Err(OwnerEpochError::InvalidInventoryEpoch);
    }
    for digest in [
        &revocation_digest,
        &inventory_manifest_digest,
        &route_set_digest,
    ] {
        require_digest(digest)?;
    }
    if targets.is_empty() {
        return Err(OwnerEpochError::EmptyOwnerSet);
    }

    let mut expected = BTreeMap::new();
    for target in targets {
        if expected
            .insert(
                target.owner.clone(),
                (target.previous_position, target.installed_position),
            )
            .is_some()
        {
            return Err(OwnerEpochError::DuplicateOwnerTarget);
        }
    }

    let mut seen = BTreeSet::new();
    for ack in acknowledgements {
        if ack.scope != *scope {
            return Err(OwnerEpochError::ScopeMismatch);
        }
        if ack.revocation_digest != revocation_digest {
            return Err(OwnerEpochError::RevocationBindingMismatch);
        }
        if ack.inventory_epoch != inventory_epoch {
            return Err(OwnerEpochError::InventoryEpochMismatch);
        }
        if ack.inventory_manifest_digest != inventory_manifest_digest {
            return Err(OwnerEpochError::InventoryManifestMismatch);
        }
        if ack.route_set_digest != route_set_digest {
            return Err(OwnerEpochError::RouteSetMismatch);
        }
        let expected_positions = expected
            .get(&ack.owner)
            .copied()
            .ok_or(OwnerEpochError::UnexpectedOwner)?;
        if !seen.insert(ack.owner.clone()) {
            return Err(OwnerEpochError::DuplicateAcknowledgement);
        }
        if (ack.previous_position, ack.installed_position) != expected_positions {
            return Err(OwnerEpochError::AcknowledgementPositionMismatch);
        }
    }

    let missing_owners: Vec<EffectOwnerId> = targets
        .iter()
        .filter(|target| !seen.contains(&target.owner))
        .map(|target| target.owner.clone())
        .collect();
    let status = if missing_owners.is_empty() {
        OwnerEpochBarrierStatus::Complete
    } else {
        OwnerEpochBarrierStatus::PendingOwnerAcknowledgements
    };

    Ok(OwnerEpochFenceRollup {
        scope: scope.clone(),
        revocation_digest,
        inventory_epoch,
        inventory_manifest_digest,
        route_set_digest,
        status,
        acknowledged_owner_count: seen.len(),
        expected_owner_count: targets.len(),
        missing_owners,
    })
}

/// Canonical digest input for the full owner receipt set. Owners are sorted and
/// duplicates rejected. Hashing/signing this byte string remains external.
pub fn canonical_owner_epoch_ack_set_bytes(
    acknowledgements: &[OwnerEpochFenceAck],
) -> Result<Vec<u8>, OwnerEpochError> {
    if acknowledgements.is_empty() {
        return Err(OwnerEpochError::EmptyOwnerSet);
    }
    let mut ordered: Vec<&OwnerEpochFenceAck> = acknowledgements.iter().collect();
    ordered.sort_by(|left, right| left.owner.cmp(&right.owner));
    for pair in ordered.windows(2) {
        if pair[0].owner == pair[1].owner {
            return Err(OwnerEpochError::DuplicateAcknowledgement);
        }
    }

    let mut bytes = b"MYC-DIR-OWNER-EPOCH-ACK-SET\0\x01".to_vec();
    let count = u32::try_from(ordered.len()).map_err(|_| OwnerEpochError::EmptyOwnerSet)?;
    bytes.extend_from_slice(&count.to_be_bytes());
    for acknowledgement in ordered {
        let encoded = acknowledgement.canonical_bytes()?;
        let length =
            u32::try_from(encoded.len()).map_err(|_| OwnerEpochError::IdentifierTooLong)?;
        bytes.extend_from_slice(&length.to_be_bytes());
        bytes.extend_from_slice(&encoded);
    }
    Ok(bytes)
}

/// Durable coordinator receipt. It can only be composed from a complete exact
/// owner-set rollup. The owner acknowledgement root and coordinator commit
/// digest are external commitments; this type does not cryptographically
/// authenticate them.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GlobalRevocationBarrierReceipt {
    scope: EffectFenceScope,
    revocation_digest: Digest,
    inventory_epoch: u64,
    inventory_manifest_digest: Digest,
    route_set_digest: Digest,
    owner_ack_set_digest: Digest,
    owner_ack_count: usize,
    coordinator_epoch: u64,
    coordinator_commit_digest: Digest,
}

impl GlobalRevocationBarrierReceipt {
    pub fn compose(
        rollup: &OwnerEpochFenceRollup,
        owner_ack_set_digest: Digest,
        coordinator_epoch: u64,
        coordinator_commit_digest: Digest,
    ) -> Result<Self, OwnerEpochError> {
        if rollup.status != OwnerEpochBarrierStatus::Complete {
            return Err(OwnerEpochError::GlobalBarrierPending);
        }
        if coordinator_epoch == 0 {
            return Err(OwnerEpochError::ZeroCoordinatorEpoch);
        }
        require_digest(&owner_ack_set_digest)?;
        require_digest(&coordinator_commit_digest)?;
        Ok(Self {
            scope: rollup.scope.clone(),
            revocation_digest: rollup.revocation_digest,
            inventory_epoch: rollup.inventory_epoch,
            inventory_manifest_digest: rollup.inventory_manifest_digest,
            route_set_digest: rollup.route_set_digest,
            owner_ack_set_digest,
            owner_ack_count: rollup.expected_owner_count,
            coordinator_epoch,
            coordinator_commit_digest,
        })
    }

    pub fn owner_ack_count(&self) -> usize {
        self.owner_ack_count
    }

    pub fn coordinator_epoch(&self) -> u64 {
        self.coordinator_epoch
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, OwnerEpochError> {
        let mut bytes = b"MYC-DIR-GLOBAL-REVOCATION-BARRIER\0\x01".to_vec();
        append_text(&mut bytes, self.scope.realm().as_str())?;
        append_text(&mut bytes, self.scope.resource().as_str())?;
        append_text(&mut bytes, self.scope.action().as_str())?;
        bytes.extend_from_slice(&self.revocation_digest);
        bytes.extend_from_slice(&self.inventory_epoch.to_be_bytes());
        bytes.extend_from_slice(&self.inventory_manifest_digest);
        bytes.extend_from_slice(&self.route_set_digest);
        bytes.extend_from_slice(&self.owner_ack_set_digest);
        let count =
            u32::try_from(self.owner_ack_count).map_err(|_| OwnerEpochError::EmptyOwnerSet)?;
        bytes.extend_from_slice(&count.to_be_bytes());
        bytes.extend_from_slice(&self.coordinator_epoch.to_be_bytes());
        bytes.extend_from_slice(&self.coordinator_commit_digest);
        Ok(bytes)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RouteAdmissionState {
    Staged,
    Active,
}

/// A new route is staged first and activated only after its owner has recovered
/// and the route is bound to the exact current owner position and inventory
/// epoch. The owner must still enforce that position atomically on every effect.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FencedRouteAdmission {
    route_id: EffectRouteId,
    owner: EffectOwnerId,
    enforcement: EffectRouteEnforcement,
    required_position: OwnerFencePosition,
    inventory_epoch: u64,
    inventory_manifest_digest: Digest,
    route_set_digest: Digest,
    state: RouteAdmissionState,
}

impl FencedRouteAdmission {
    pub fn stage(
        route_id: EffectRouteId,
        owner: EffectOwnerId,
        enforcement: EffectRouteEnforcement,
        required_position: OwnerFencePosition,
        inventory_epoch: u64,
        inventory_manifest_digest: Digest,
        route_set_digest: Digest,
    ) -> Result<Self, OwnerEpochError> {
        if inventory_epoch == 0 {
            return Err(OwnerEpochError::InvalidInventoryEpoch);
        }
        require_digest(&inventory_manifest_digest)?;
        require_digest(&route_set_digest)?;
        Ok(Self {
            route_id,
            owner,
            enforcement,
            required_position,
            inventory_epoch,
            inventory_manifest_digest,
            route_set_digest,
            state: RouteAdmissionState::Staged,
        })
    }

    pub fn route_id(&self) -> &EffectRouteId {
        &self.route_id
    }

    pub fn state(&self) -> RouteAdmissionState {
        self.state
    }

    pub fn activate(
        &mut self,
        owner_state: &OwnerEpochState,
        current_inventory_epoch: u64,
        current_inventory_manifest_digest: Digest,
        current_route_set_digest: Digest,
    ) -> Result<(), OwnerEpochError> {
        if self.state != RouteAdmissionState::Staged {
            return Err(OwnerEpochError::RouteAlreadyActive);
        }
        if self.enforcement != EffectRouteEnforcement::AtomicOwnerFence {
            return Err(OwnerEpochError::NonFenceableRoute);
        }
        if owner_state.owner != self.owner {
            return Err(OwnerEpochError::OwnerMismatch);
        }
        if !owner_state.recovery_ready {
            return Err(OwnerEpochError::RecoveryRequired);
        }
        if !owner_state.current_is_durable() {
            return Err(OwnerEpochError::OwnerFenceNotDurablyReadBack);
        }
        if current_inventory_epoch != self.inventory_epoch {
            return Err(OwnerEpochError::InventoryEpochMismatch);
        }
        if current_inventory_manifest_digest != self.inventory_manifest_digest {
            return Err(OwnerEpochError::InventoryManifestMismatch);
        }
        if current_route_set_digest != self.route_set_digest {
            return Err(OwnerEpochError::RouteSetMismatch);
        }
        if owner_state.current != self.required_position {
            return Err(OwnerEpochError::RouteFencePositionMismatch);
        }
        self.state = RouteAdmissionState::Active;
        Ok(())
    }

    /// Existing active routes may retain an old capability after a new
    /// revocation. This assessment ensures they cannot treat route activation
    /// as continuing authorization; the resource owner must reject the stale
    /// position at mutation time.
    pub fn assess_current_owner_position(
        &self,
        owner_state: &OwnerEpochState,
    ) -> OwnerPositionAssessment {
        if self.state != RouteAdmissionState::Active || owner_state.owner != self.owner {
            return OwnerPositionAssessment::RecoveryRequired;
        }
        owner_state.assess_position(self.required_position)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum OwnerEpochError {
    EmptyIdentifier,
    IdentifierTooLong,
    ZeroDigest,
    ZeroPosition,
    ZeroCoordinatorEpoch,
    InvalidInventoryEpoch,
    GenerationExhausted,
    GenerationBelowHighWater,
    OwnerEpochNotAdvanced,
    RecoveryRequired,
    OwnerMismatch,
    NoPendingFence,
    AcknowledgementPositionNotMonotonic,
    AcknowledgementPositionMismatch,
    ScopeMismatch,
    RevocationBindingMismatch,
    InventoryEpochMismatch,
    InventoryManifestMismatch,
    RouteSetMismatch,
    EmptyOwnerSet,
    DuplicateOwnerTarget,
    DuplicateAcknowledgement,
    UnexpectedOwner,
    GlobalBarrierPending,
    NonFenceableRoute,
    RouteAlreadyActive,
    RouteFencePositionMismatch,
    OwnerFenceNotDurablyReadBack,
    AcknowledgementEvidenceMismatch,
}

impl std::fmt::Display for OwnerEpochError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for OwnerEpochError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::effect_inventory::EffectRouteEnforcement;

    fn owner(name: &str) -> EffectOwnerId {
        EffectOwnerId::new(name).unwrap()
    }

    fn scope() -> EffectFenceScope {
        EffectFenceScope::new(
            crate::RealmId::new("realm-a").unwrap(),
            crate::ResourceId::new("ledger-a").unwrap(),
            crate::ActionId::new("post-entry").unwrap(),
        )
    }

    fn position(epoch: u64, generation: u64) -> OwnerFencePosition {
        OwnerFencePosition::new(epoch, generation).unwrap()
    }

    fn owner_state(name: &str) -> OwnerEpochState {
        OwnerEpochState::from_verified_bootstrap(
            owner(name),
            position(1, 1),
            [1; 32],
            [2; 32],
        )
        .unwrap()
    }

    fn target(name: &str, previous: OwnerFencePosition, installed: OwnerFencePosition) -> OwnerEpochFenceTarget {
        OwnerEpochFenceTarget::new(owner(name), previous, installed).unwrap()
    }

    fn ack(name: &str, previous: OwnerFencePosition, installed: OwnerFencePosition) -> OwnerEpochFenceAck {
        OwnerEpochFenceAck::new(
            scope(),
            owner(name),
            previous,
            installed,
            [3; 32],
            7,
            [4; 32],
            [5; 32],
            [6; 32],
            [7; 32],
        )
        .unwrap()
    }

    fn two_owner_inputs() -> (Vec<OwnerEpochFenceTarget>, Vec<OwnerEpochFenceAck>) {
        let targets = vec![
            target("db-a", position(1, 1), position(1, 2)),
            target("queue-b", position(4, 8), position(4, 9)),
        ];
        let acknowledgements = vec![
            ack("db-a", position(1, 1), position(1, 2)),
            ack("queue-b", position(4, 8), position(4, 9)),
        ];
        (targets, acknowledgements)
    }

    fn rollup(
        targets: &[OwnerEpochFenceTarget],
        acknowledgements: &[OwnerEpochFenceAck],
    ) -> OwnerEpochFenceRollup {
        assess_owner_epoch_fence_acks(
            &scope(),
            [3; 32],
            7,
            [4; 32],
            [5; 32],
            targets,
            acknowledgements,
        )
        .unwrap()
    }

    #[test]
    fn owner_epoch_and_generation_must_both_be_positive() {
        assert_eq!(OwnerFencePosition::new(0, 1), Err(OwnerEpochError::ZeroPosition));
        assert_eq!(OwnerFencePosition::new(1, 0), Err(OwnerEpochError::ZeroPosition));
    }

    #[test]
    fn old_epoch_is_rejected_even_when_generation_matches() {
        let state = OwnerEpochState::from_verified_bootstrap(
            owner("db-a"),
            position(2, 1),
            [1; 32],
            [2; 32],
        )
        .unwrap();
        assert_eq!(
            state.assess_position(position(1, 1)),
            OwnerPositionAssessment::StaleOwnerEpoch
        );
        assert_eq!(
            state.assess_position(position(2, 1)),
            OwnerPositionAssessment::Current
        );
    }

    #[test]
    fn recovery_fails_closed_until_high_water_reconciliation() {
        let high_water = OwnerHighWaterMark::new(owner("db-a"), position(3, 9), [8; 32]).unwrap();
        let mut state = OwnerEpochState::from_unverified_restore(
            owner("db-a"),
            position(2, 4),
            high_water,
        )
        .unwrap();
        assert!(!state.recovery_ready());
        assert_eq!(
            state.assess_position(position(2, 4)),
            OwnerPositionAssessment::RecoveryRequired
        );
        assert_eq!(
            state.install_next_generation(),
            Err(OwnerEpochError::RecoveryRequired)
        );
    }

    #[test]
    fn recovery_must_advance_epoch_and_not_regress_generation() {
        let high_water = OwnerHighWaterMark::new(owner("db-a"), position(3, 9), [8; 32]).unwrap();
        let mut state = OwnerEpochState::from_unverified_restore(
            owner("db-a"),
            position(2, 4),
            high_water,
        )
        .unwrap();
        assert_eq!(
            state.reconcile_recovery(3, 9, [9; 32], [10; 32]),
            Err(OwnerEpochError::OwnerEpochNotAdvanced)
        );
        assert_eq!(
            state.reconcile_recovery(4, 8, [9; 32], [10; 32]),
            Err(OwnerEpochError::GenerationBelowHighWater)
        );
        let recovered = state.reconcile_recovery(4, 9, [9; 32], [10; 32]).unwrap();
        assert_eq!(recovered, position(4, 9));
        assert!(state.recovery_ready());
    }

    #[test]
    fn fence_install_increments_generation_without_wrapping() {
        let mut state = owner_state("db-a");
        assert_eq!(state.install_next_generation().unwrap(), position(1, 2));
        state.install_next_generation().unwrap();
        state.current = position(1, u64::MAX);
        assert_eq!(
            state.install_next_generation(),
            Err(OwnerEpochError::GenerationExhausted)
        );
    }

    #[test]
    fn acknowledgement_must_advance_exact_current_position() {
        let mut state = owner_state("db-a");
        state.install_next_generation().unwrap();
        let ack = ack("db-a", position(1, 1), position(1, 2));
        state
            .record_durable_commit_readback(position(1, 2), [6; 32], [7; 32])
            .unwrap();
        let ack = OwnerEpochFenceAck::new(
            scope(),
            owner("db-a"),
            position(1, 1),
            position(1, 2),
            [3; 32],
            7,
            [4; 32],
            [5; 32],
            [6; 32],
            [7; 32],
        )
        .unwrap();
        state
            .record_acknowledgement(
                &ack,
                [11; 32],
                &scope(),
                [3; 32],
                7,
                [4; 32],
                [5; 32],
            )
            .unwrap();
        assert_eq!(state.high_water().position(), position(1, 2));
        assert_eq!(
            state.record_acknowledgement(
                &ack,
                [12; 32],
                &scope(),
                [3; 32],
                7,
                [4; 32],
                [5; 32],
            ),
            Err(OwnerEpochError::AcknowledgementPositionMismatch)
        );
    }

    #[test]
    fn acknowledgement_set_requires_exact_scope_inventory_and_owner_positions() {
        let (targets, acks) = two_owner_inputs();
        let complete = rollup(&targets, &acks);
        assert_eq!(complete.status(), OwnerEpochBarrierStatus::Complete);
        assert_eq!(complete.acknowledged_owner_count(), 2);
        assert_eq!(complete.expected_owner_count(), 2);
        assert!(complete.missing_owners().is_empty());
    }

    #[test]
    fn missing_owner_keeps_global_barrier_pending() {
        let (targets, acks) = two_owner_inputs();
        let pending = rollup(&targets, &acks[..1]);
        assert_eq!(
            pending.status(),
            OwnerEpochBarrierStatus::PendingOwnerAcknowledgements
        );
        assert_eq!(pending.missing_owners(), &[owner("queue-b")]);
        assert_eq!(
            GlobalRevocationBarrierReceipt::compose(&pending, [12; 32], 1, [13; 32]),
            Err(OwnerEpochError::GlobalBarrierPending)
        );
    }

    #[test]
    fn duplicate_or_unexpected_owner_ack_is_rejected() {
        let (targets, acks) = two_owner_inputs();
        assert_eq!(
            assess_owner_epoch_fence_acks(
                &scope(), [3; 32], 7, [4; 32], [5; 32], &targets, &[acks[0].clone(), acks[0].clone()]
            ),
            Err(OwnerEpochError::DuplicateAcknowledgement)
        );
        let unexpected = ack("other", position(1, 1), position(1, 2));
        assert_eq!(
            assess_owner_epoch_fence_acks(
                &scope(), [3; 32], 7, [4; 32], [5; 32], &targets, &[unexpected]
            ),
            Err(OwnerEpochError::UnexpectedOwner)
        );
    }

    #[test]
    fn mismatched_owner_epoch_or_generation_ack_is_rejected() {
        let (targets, _) = two_owner_inputs();
        let stale_epoch = ack("db-a", position(0 + 1, 1), position(1, 2));
        let substituted_epoch = ack("db-a", position(2, 1), position(2, 2));
        assert_eq!(
            assess_owner_epoch_fence_acks(&scope(), [3; 32], 7, [4; 32], [5; 32], &targets, &[stale_epoch]),
            Err(OwnerEpochError::AcknowledgementPositionMismatch)
        );
        assert_eq!(
            assess_owner_epoch_fence_acks(&scope(), [3; 32], 7, [4; 32], [5; 32], &targets, &[substituted_epoch]),
            Err(OwnerEpochError::AcknowledgementPositionMismatch)
        );
    }

    #[test]
    fn canonical_ack_set_is_order_independent_and_duplicate_sensitive() {
        let (_, acks) = two_owner_inputs();
        let reversed = vec![acks[1].clone(), acks[0].clone()];
        assert_eq!(
            canonical_owner_epoch_ack_set_bytes(&acks).unwrap(),
            canonical_owner_epoch_ack_set_bytes(&reversed).unwrap()
        );
        assert_eq!(
            canonical_owner_epoch_ack_set_bytes(&[acks[0].clone(), acks[0].clone()]),
            Err(OwnerEpochError::DuplicateAcknowledgement)
        );
    }

    #[test]
    fn global_receipt_requires_all_owner_acks_and_binds_coordinator_epoch() {
        let (targets, acks) = two_owner_inputs();
        let complete = rollup(&targets, &acks);
        let bytes = canonical_owner_epoch_ack_set_bytes(&acks).unwrap();
        assert!(!bytes.is_empty());
        let receipt =
            GlobalRevocationBarrierReceipt::compose(&complete, [14; 32], 3, [15; 32]).unwrap();
        assert_eq!(receipt.owner_ack_count(), 2);
        assert_eq!(receipt.coordinator_epoch(), 3);
        assert!(!receipt.canonical_bytes().unwrap().is_empty());
    }

    #[test]
    fn new_route_cannot_activate_before_owner_fence_adoption() {
        let owner_state = owner_state("db-a");
        let mut route = FencedRouteAdmission::stage(
            EffectRouteId::new("new-worker-route").unwrap(),
            owner("db-a"),
            EffectRouteEnforcement::AtomicOwnerFence,
            position(1, 2),
            8,
            [44; 32],
            [45; 32],
        )
        .unwrap();
        assert_eq!(
            route.activate(&owner_state, 8, [44; 32], [45; 32]),
            Err(OwnerEpochError::RouteFencePositionMismatch)
        );
        assert_eq!(route.state(), RouteAdmissionState::Staged);
    }

    #[test]
    fn new_route_activates_only_after_current_owner_fence_and_inventory_match() {
        let mut owner_state = owner_state("db-a");
        owner_state.install_next_generation().unwrap();
        assert_eq!(
            owner_state.record_durable_commit_readback(position(1, 2), [46; 32], [47; 32]),
            Ok(())
        );
        let mut route = FencedRouteAdmission::stage(
            EffectRouteId::new("new-worker-route").unwrap(),
            owner("db-a"),
            EffectRouteEnforcement::AtomicOwnerFence,
            position(1, 2),
            8,
            [44; 32],
            [45; 32],
        )
        .unwrap();
        assert_eq!(
            route.activate(&owner_state, 7, [44; 32], [45; 32]),
            Err(OwnerEpochError::InventoryEpochMismatch)
        );
        assert_eq!(
            route.activate(&owner_state, 8, [40; 32], [45; 32]),
            Err(OwnerEpochError::InventoryManifestMismatch)
        );
        assert_eq!(
            route.activate(&owner_state, 8, [44; 32], [40; 32]),
            Err(OwnerEpochError::RouteSetMismatch)
        );
        route.activate(&owner_state, 8, [44; 32], [45; 32]).unwrap();
        assert_eq!(route.state(), RouteAdmissionState::Active);
    }

    #[test]
    fn non_fenceable_route_cannot_be_activated_for_strict_execution() {
        let owner_state = owner_state("db-a");
        let mut route = FencedRouteAdmission::stage(
            EffectRouteId::new("signal-only-route").unwrap(),
            owner("db-a"),
            EffectRouteEnforcement::CaepSignalOnly,
            position(1, 1),
            8,
            [44; 32],
            [45; 32],
        )
        .unwrap();
        assert_eq!(
            route.activate(&owner_state, 8, [44; 32], [45; 32]),
            Err(OwnerEpochError::NonFenceableRoute)
        );
    }

    #[test]
    fn active_route_capability_becomes_stale_after_owner_advances() {
        let mut owner_state = owner_state("db-a");
        let mut route = FencedRouteAdmission::stage(
            EffectRouteId::new("worker-route").unwrap(),
            owner("db-a"),
            EffectRouteEnforcement::AtomicOwnerFence,
            position(1, 1),
            8,
            [44; 32],
            [45; 32],
        )
        .unwrap();
        route.activate(&owner_state, 8, [44; 32], [45; 32]).unwrap();
        owner_state.install_next_generation().unwrap();
        assert_eq!(
            route.assess_current_owner_position(&owner_state),
            OwnerPositionAssessment::StaleGeneration
        );
    }
}
