//! Stable-frontier, safe-reclamation, and cold-start integrity reference model.
//!
//! Status: ReferenceModelOnly.
//! Reclamation is permitted only from a closed semantic scope with explicit
//! frontier coverage. Elapsed time, arrival order, or local convenience never
//! substitutes for causal coverage.

use crate::no_resurrection::SemanticTombstone;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const STABLE_FRONTIER_PROFILE_ID: &str = "INTEGRAL-RETENTION-REF-001";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ParticipantRoleV1 {
    AuthorityBearing,
    MirrorOnly,
    ObserverOnly,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ParticipantStateV1 {
    Active,
    Fenced,
    Retired,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ScopeParticipantV1 {
    pub participant_id: String,
    pub role: ParticipantRoleV1,
    pub state: ParticipantStateV1,
    pub membership_epoch: u64,
    pub semantic_environment_root: String,
    pub membership_evidence_root: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticStabilityScopeV1 {
    pub scope_id: String,
    pub semantic_environment_root: String,
    pub membership_epoch: u64,
    pub participants: BTreeMap<String, ScopeParticipantV1>,
    pub unknown_authority_members: BTreeSet<String>,
    pub scope_commitment: String,
}

impl SemanticStabilityScopeV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.scope_id.is_empty()
            && !self.semantic_environment_root.is_empty()
            && !self.scope_commitment.is_empty()
            && self
                .participants
                .iter()
                .all(|(id, participant)| {
                    id == &participant.participant_id
                        && participant.membership_epoch == self.membership_epoch
                        && participant.semantic_environment_root == self.semantic_environment_root
                        && !participant.membership_evidence_root.is_empty()
                })
            && self
                .unknown_authority_members
                .iter()
                .all(|id| !self.participants.contains_key(id))
    }

    pub fn authority_members(&self) -> impl Iterator<Item = (&String, &ScopeParticipantV1)> {
        self.participants.iter().filter(|(_, participant)| {
            participant.role == ParticipantRoleV1::AuthorityBearing
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FrontierCoverageStatusV1 {
    Covered,
    Fenced,
    Retired,
    Stale,
    Missing,
    Conflicting,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FrontierCoverageV1 {
    pub participant_id: String,
    pub membership_epoch: u64,
    pub semantic_environment_root: String,
    pub observed_frontier_root: Option<String>,
    pub status: FrontierCoverageStatusV1,
    pub evidence_root: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StableFrontierDispositionV1 {
    Stable,
    BlockedUnknownAuthority,
    BlockedMembershipEpoch,
    BlockedMissingCoverage,
    BlockedStaleCoverage,
    BlockedFenceEvidence,
    Conflict,
    InsufficientEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StableFrontierCertificateV1 {
    pub certificate_id: String,
    pub scope_id: String,
    pub semantic_environment_root: String,
    pub membership_epoch: u64,
    pub candidate_frontier_root: String,
    pub covered_frontier_roots: BTreeSet<String>,
    pub coverage: BTreeMap<String, FrontierCoverageV1>,
    pub certificate_commitment: String,
    pub claim_ceiling: String,
}

pub fn assess_stable_frontier(
    scope: &SemanticStabilityScopeV1,
    certificate: &StableFrontierCertificateV1,
) -> StableFrontierDispositionV1 {
    if !scope.structurally_valid()
        || certificate.certificate_id.is_empty()
        || certificate.candidate_frontier_root.is_empty()
        || certificate.certificate_commitment.is_empty()
        || certificate.claim_ceiling.is_empty()
    {
        return StableFrontierDispositionV1::InsufficientEvidence;
    }
    if !scope.unknown_authority_members.is_empty() {
        return StableFrontierDispositionV1::BlockedUnknownAuthority;
    }
    if certificate.scope_id != scope.scope_id
        || certificate.semantic_environment_root != scope.semantic_environment_root
    {
        return StableFrontierDispositionV1::Conflict;
    }
    if certificate.membership_epoch != scope.membership_epoch {
        return StableFrontierDispositionV1::BlockedMembershipEpoch;
    }
    if !certificate
        .covered_frontier_roots
        .contains(&certificate.candidate_frontier_root)
    {
        return StableFrontierDispositionV1::InsufficientEvidence;
    }
    if certificate
        .coverage
        .keys()
        .any(|id| !scope.participants.contains_key(id))
    {
        return StableFrontierDispositionV1::Conflict;
    }

    for (id, participant) in scope.authority_members() {
        let Some(coverage) = certificate.coverage.get(id) else {
            return StableFrontierDispositionV1::BlockedMissingCoverage;
        };
        if coverage.participant_id.as_str() != id.as_str()
            || coverage.membership_epoch != scope.membership_epoch
            || coverage.semantic_environment_root != scope.semantic_environment_root
            || coverage.evidence_root.is_empty()
        {
            return StableFrontierDispositionV1::BlockedMembershipEpoch;
        }

        match participant.state {
            ParticipantStateV1::Active => {
                if coverage.status != FrontierCoverageStatusV1::Covered {
                    return match coverage.status {
                        FrontierCoverageStatusV1::Stale => {
                            StableFrontierDispositionV1::BlockedStaleCoverage
                        }
                        FrontierCoverageStatusV1::Missing => {
                            StableFrontierDispositionV1::BlockedMissingCoverage
                        }
                        FrontierCoverageStatusV1::Conflicting => {
                            StableFrontierDispositionV1::Conflict
                        }
                        FrontierCoverageStatusV1::Fenced | FrontierCoverageStatusV1::Retired => {
                            StableFrontierDispositionV1::BlockedFenceEvidence
                        }
                        FrontierCoverageStatusV1::Covered => unreachable!(),
                    };
                }
                if coverage.observed_frontier_root.as_deref()
                    != Some(certificate.candidate_frontier_root.as_str())
                {
                    return StableFrontierDispositionV1::BlockedStaleCoverage;
                }
            }
            ParticipantStateV1::Fenced => {
                if coverage.status != FrontierCoverageStatusV1::Fenced {
                    return StableFrontierDispositionV1::BlockedFenceEvidence;
                }
            }
            ParticipantStateV1::Retired => {
                if coverage.status != FrontierCoverageStatusV1::Retired {
                    return StableFrontierDispositionV1::BlockedFenceEvidence;
                }
            }
        }
    }

    StableFrontierDispositionV1::Stable
}

pub fn stable_frontier_covers_tombstone(
    certificate: &StableFrontierCertificateV1,
    tombstone: &SemanticTombstone,
) -> bool {
    certificate
        .covered_frontier_roots
        .contains(&tombstone.causal_frontier_root)
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RetentionBoundaryV1 {
    pub stable_frontier_certificate_id: String,
    pub stable_frontier_root: String,
    pub history_inventory_closed: bool,
    pub known_history_roots: BTreeSet<String>,
    pub reclaimable_history_roots: BTreeSet<String>,
    pub retained_history_roots: BTreeSet<String>,
    pub tombstone_inventory_closed: bool,
    pub known_tombstone_ids: BTreeSet<String>,
    pub reclaimable_tombstone_ids: BTreeSet<String>,
    pub retained_tombstone_ids: BTreeSet<String>,
    pub boundary_commitment: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RetentionDispositionV1 {
    SafeToPrune,
    BlockedWithoutStableFrontier,
    BlockedIncompleteHistoryInventory,
    BlockedIncompleteTombstoneInventory,
    BlockedUncoveredTombstone,
    BlockedUnclassifiedHistory,
    BlockedUnclassifiedTombstone,
    Conflict,
    InsufficientEvidence,
}

fn exact_partition(
    known: &BTreeSet<String>,
    left: &BTreeSet<String>,
    right: &BTreeSet<String>,
) -> bool {
    left.is_disjoint(right)
        && left.is_subset(known)
        && right.is_subset(known)
        && left.union(right).cloned().collect::<BTreeSet<_>>() == known.clone()
}

pub fn assess_retention_boundary(
    boundary: &RetentionBoundaryV1,
    scope: &SemanticStabilityScopeV1,
    certificate: &StableFrontierCertificateV1,
    tombstones: &BTreeMap<String, SemanticTombstone>,
) -> RetentionDispositionV1 {
    if assess_stable_frontier(scope, certificate) != StableFrontierDispositionV1::Stable {
        return RetentionDispositionV1::BlockedWithoutStableFrontier;
    }
    if boundary.stable_frontier_certificate_id != certificate.certificate_id
        || boundary.stable_frontier_root != certificate.candidate_frontier_root
        || boundary.boundary_commitment.is_empty()
    {
        return RetentionDispositionV1::Conflict;
    }
    if !boundary.history_inventory_closed {
        return RetentionDispositionV1::BlockedIncompleteHistoryInventory;
    }
    if !boundary.tombstone_inventory_closed {
        return RetentionDispositionV1::BlockedIncompleteTombstoneInventory;
    }
    if !exact_partition(
        &boundary.known_history_roots,
        &boundary.reclaimable_history_roots,
        &boundary.retained_history_roots,
    ) {
        return RetentionDispositionV1::BlockedUnclassifiedHistory;
    }
    if !exact_partition(
        &boundary.known_tombstone_ids,
        &boundary.reclaimable_tombstone_ids,
        &boundary.retained_tombstone_ids,
    ) {
        return RetentionDispositionV1::BlockedUnclassifiedTombstone;
    }

    for id in &boundary.known_tombstone_ids {
        if !tombstones.contains_key(id) {
            return RetentionDispositionV1::InsufficientEvidence;
        }
    }
    for id in &boundary.reclaimable_tombstone_ids {
        let Some(tombstone) = tombstones.get(id) else {
            return RetentionDispositionV1::InsufficientEvidence;
        };
        if !stable_frontier_covers_tombstone(certificate, tombstone) {
            return RetentionDispositionV1::BlockedUncoveredTombstone;
        }
    }

    RetentionDispositionV1::SafeToPrune
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PruningReceiptV1 {
    pub pruning_id: String,
    pub scope_id: String,
    pub semantic_environment_root: String,
    pub membership_epoch: u64,
    pub source_snapshot_root: String,
    pub retention_boundary_commitment: String,
    pub reclaimed_history_roots: BTreeSet<String>,
    pub reclaimed_tombstone_ids: BTreeSet<String>,
    pub resulting_snapshot_root: String,
    pub claim_ceiling: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PruningDispositionV1 {
    Accepted,
    Blocked,
    Conflict,
    InsufficientEvidence,
}

pub fn assess_pruning_receipt(
    receipt: &PruningReceiptV1,
    boundary: &RetentionBoundaryV1,
    scope: &SemanticStabilityScopeV1,
    certificate: &StableFrontierCertificateV1,
    tombstones: &BTreeMap<String, SemanticTombstone>,
) -> PruningDispositionV1 {
    if receipt.pruning_id.is_empty()
        || receipt.source_snapshot_root.is_empty()
        || receipt.resulting_snapshot_root.is_empty()
        || receipt.claim_ceiling.is_empty()
    {
        return PruningDispositionV1::InsufficientEvidence;
    }
    if receipt.scope_id != scope.scope_id
        || receipt.semantic_environment_root != scope.semantic_environment_root
        || receipt.membership_epoch != scope.membership_epoch
        || receipt.retention_boundary_commitment != boundary.boundary_commitment
    {
        return PruningDispositionV1::Conflict;
    }
    if assess_retention_boundary(boundary, scope, certificate, tombstones)
        != RetentionDispositionV1::SafeToPrune
    {
        return PruningDispositionV1::Blocked;
    }
    if receipt.reclaimed_history_roots != boundary.reclaimable_history_roots
        || receipt.reclaimed_tombstone_ids != boundary.reclaimable_tombstone_ids
    {
        return PruningDispositionV1::Conflict;
    }
    PruningDispositionV1::Accepted
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ColdStartManifestV1 {
    pub node_id: String,
    pub incarnation_id: String,
    pub semantic_environment_root: String,
    pub membership_epoch: u64,
    pub snapshot_root: String,
    pub snapshot_frontier_root: String,
    pub required_history_roots: BTreeSet<String>,
    pub retained_tombstone_ids: BTreeSet<String>,
    pub reconstruction_profile_id: String,
    pub normative_state_root: String,
    pub manifest_commitment: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReconstructionDispositionV1 {
    Ready,
    BlockedMissingHistory,
    BlockedMissingTombstone,
    InsufficientEvidence,
}

pub fn assess_cold_start_reconstruction(
    manifest: &ColdStartManifestV1,
    available_history_roots: &BTreeSet<String>,
    available_tombstone_ids: &BTreeSet<String>,
) -> ReconstructionDispositionV1 {
    if manifest.node_id.is_empty()
        || manifest.incarnation_id.is_empty()
        || manifest.semantic_environment_root.is_empty()
        || manifest.snapshot_root.is_empty()
        || manifest.snapshot_frontier_root.is_empty()
        || manifest.reconstruction_profile_id.is_empty()
        || manifest.normative_state_root.is_empty()
        || manifest.manifest_commitment.is_empty()
    {
        return ReconstructionDispositionV1::InsufficientEvidence;
    }
    if !manifest
        .required_history_roots
        .is_subset(available_history_roots)
    {
        return ReconstructionDispositionV1::BlockedMissingHistory;
    }
    if !manifest
        .retained_tombstone_ids
        .is_subset(available_tombstone_ids)
    {
        return ReconstructionDispositionV1::BlockedMissingTombstone;
    }
    ReconstructionDispositionV1::Ready
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReconstructionReceiptV1 {
    pub node_id: String,
    pub incarnation_id: String,
    pub semantic_environment_root: String,
    pub snapshot_root: String,
    pub source_frontier_root: String,
    pub reconstructed_state_root: String,
    pub retained_tombstone_ids: BTreeSet<String>,
    pub claim_ceiling: String,
}

pub fn reconstruction_matches_manifest(
    receipt: &ReconstructionReceiptV1,
    manifest: &ColdStartManifestV1,
) -> bool {
    receipt.node_id == manifest.node_id
        && receipt.incarnation_id == manifest.incarnation_id
        && receipt.semantic_environment_root == manifest.semantic_environment_root
        && receipt.snapshot_root == manifest.snapshot_root
        && receipt.source_frontier_root == manifest.snapshot_frontier_root
        && receipt.reconstructed_state_root == manifest.normative_state_root
        && receipt.retained_tombstone_ids == manifest.retained_tombstone_ids
        && !receipt.claim_ceiling.is_empty()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RejoinDispositionV1 {
    Accepted,
    BlockedUnknownParticipant,
    BlockedFenced,
    BlockedRetired,
    BlockedStaleMembership,
    BlockedStaleFrontier,
    BlockedEnvironment,
    InsufficientEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RejoinAssessmentV1 {
    pub node_id: String,
    pub incarnation_id: String,
    pub presented_membership_epoch: u64,
    pub presented_frontier_root: String,
    pub disposition: RejoinDispositionV1,
    pub claim_ceiling: String,
}

pub fn assess_rejoin(
    scope: &SemanticStabilityScopeV1,
    node_id: &str,
    incarnation_id: &str,
    semantic_environment_root: &str,
    presented_membership_epoch: u64,
    presented_frontier_root: &str,
    normative_frontier_root: &str,
) -> RejoinAssessmentV1 {
    let disposition = if node_id.is_empty()
        || incarnation_id.is_empty()
        || semantic_environment_root.is_empty()
        || presented_frontier_root.is_empty()
        || normative_frontier_root.is_empty()
    {
        RejoinDispositionV1::InsufficientEvidence
    } else if semantic_environment_root != scope.semantic_environment_root {
        RejoinDispositionV1::BlockedEnvironment
    } else {
        match scope.participants.get(node_id) {
            None => RejoinDispositionV1::BlockedUnknownParticipant,
            Some(participant) => match participant.state {
                ParticipantStateV1::Fenced => RejoinDispositionV1::BlockedFenced,
                ParticipantStateV1::Retired => RejoinDispositionV1::BlockedRetired,
                ParticipantStateV1::Active => {
                    if presented_membership_epoch != scope.membership_epoch {
                        RejoinDispositionV1::BlockedStaleMembership
                    } else if presented_frontier_root != normative_frontier_root {
                        RejoinDispositionV1::BlockedStaleFrontier
                    } else {
                        RejoinDispositionV1::Accepted
                    }
                }
            },
        }
    };

    RejoinAssessmentV1 {
        node_id: node_id.into(),
        incarnation_id: incarnation_id.into(),
        presented_membership_epoch,
        presented_frontier_root: presented_frontier_root.into(),
        disposition,
        claim_ceiling: "Rejoin gate only; no durable-storage or production-safety claim.".into(),
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConservedClaimStateV1 {
    pub authority_claims: BTreeSet<String>,
    pub capacity_claims: BTreeSet<String>,
    pub consent_claims: BTreeSet<String>,
}

pub fn reclamation_cannot_create_claims(
    before: &ConservedClaimStateV1,
    after: &ConservedClaimStateV1,
) -> bool {
    after.authority_claims.is_subset(&before.authority_claims)
        && after.capacity_claims.is_subset(&before.capacity_claims)
        && after.consent_claims.is_subset(&before.consent_claims)
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RetentionWitnessV1 {
    pub certificate_id: String,
    pub boundary_commitment: Option<String>,
    pub disposition: RetentionDispositionV1,
    pub claim_ceiling: String,
}

pub fn retention_witness(
    scope: &SemanticStabilityScopeV1,
    certificate: &StableFrontierCertificateV1,
    boundary: Option<&RetentionBoundaryV1>,
    tombstones: &BTreeMap<String, SemanticTombstone>,
) -> RetentionWitnessV1 {
    let disposition = boundary.map_or(
        RetentionDispositionV1::BlockedWithoutStableFrontier,
        |boundary| assess_retention_boundary(boundary, scope, certificate, tombstones),
    );
    RetentionWitnessV1 {
        certificate_id: certificate.certificate_id.clone(),
        boundary_commitment: boundary.map(|b| b.boundary_commitment.clone()),
        disposition,
        claim_ceiling: "Stable-frontier and reclamation reference semantics only; no durable-storage, legal-deletion, privacy, or production-safety claim.".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::no_resurrection::TombstoneReason;

    fn scope() -> SemanticStabilityScopeV1 {
        let mut participants = BTreeMap::new();
        for id in ["node-a", "node-b"] {
            participants.insert(
                id.into(),
                ScopeParticipantV1 {
                    participant_id: id.into(),
                    role: ParticipantRoleV1::AuthorityBearing,
                    state: ParticipantStateV1::Active,
                    membership_epoch: 7,
                    semantic_environment_root: "env-1".into(),
                    membership_evidence_root: format!("member-{id}-7"),
                },
            );
        }
        SemanticStabilityScopeV1 {
            scope_id: "scope-1".into(),
            semantic_environment_root: "env-1".into(),
            membership_epoch: 7,
            participants,
            unknown_authority_members: BTreeSet::new(),
            scope_commitment: "scope-commit-1".into(),
        }
    }

    fn certificate() -> StableFrontierCertificateV1 {
        let mut coverage = BTreeMap::new();
        for id in ["node-a", "node-b"] {
            coverage.insert(
                id.into(),
                FrontierCoverageV1 {
                    participant_id: id.into(),
                    membership_epoch: 7,
                    semantic_environment_root: "env-1".into(),
                    observed_frontier_root: Some("frontier-7".into()),
                    status: FrontierCoverageStatusV1::Covered,
                    evidence_root: format!("ack-{id}-7"),
                },
            );
        }
        StableFrontierCertificateV1 {
            certificate_id: "cert-7".into(),
            scope_id: "scope-1".into(),
            semantic_environment_root: "env-1".into(),
            membership_epoch: 7,
            candidate_frontier_root: "frontier-7".into(),
            covered_frontier_roots: BTreeSet::from(["frontier-7".into(), "frontier-6".into()]),
            coverage,
            certificate_commitment: "cert-commit-7".into(),
            claim_ceiling: "ReferenceModelOnly".into(),
        }
    }

    fn tombstone() -> SemanticTombstone {
        SemanticTombstone {
            tombstone_id: "tomb-1".into(),
            lineage_id: "lineage-1".into(),
            retired_generation_id: "gen-1".into(),
            retired_creation_event_id: "event-gen-1".into(),
            causal_frontier_root: "frontier-6".into(),
            reason: TombstoneReason::Revoked,
            provenance_root: "prov-1".into(),
        }
    }

    fn boundary() -> RetentionBoundaryV1 {
        RetentionBoundaryV1 {
            stable_frontier_certificate_id: "cert-7".into(),
            stable_frontier_root: "frontier-7".into(),
            history_inventory_closed: true,
            known_history_roots: BTreeSet::from(["hist-1".into(), "hist-2".into()]),
            reclaimable_history_roots: BTreeSet::from(["hist-1".into()]),
            retained_history_roots: BTreeSet::from(["hist-2".into()]),
            tombstone_inventory_closed: true,
            known_tombstone_ids: BTreeSet::from(["tomb-1".into()]),
            reclaimable_tombstone_ids: BTreeSet::from(["tomb-1".into()]),
            retained_tombstone_ids: BTreeSet::new(),
            boundary_commitment: "boundary-7".into(),
        }
    }

    #[test]
    fn stable_frontier_requires_all_active_authority_members() {
        let mut cert = certificate();
        cert.coverage.remove("node-b");
        assert_eq!(
            assess_stable_frontier(&scope(), &cert),
            StableFrontierDispositionV1::BlockedMissingCoverage
        );
    }

    #[test]
    fn unknown_offline_authority_blocks_stability() {
        let mut scoped = scope();
        scoped.unknown_authority_members.insert("node-c".into());
        assert_eq!(
            assess_stable_frontier(&scoped, &certificate()),
            StableFrontierDispositionV1::BlockedUnknownAuthority
        );
    }

    #[test]
    fn stale_peer_frontier_cannot_claim_stability() {
        let mut cert = certificate();
        cert.coverage
            .get_mut("node-b")
            .unwrap()
            .observed_frontier_root = Some("frontier-6".into());
        assert_eq!(
            assess_stable_frontier(&scope(), &cert),
            StableFrontierDispositionV1::BlockedStaleCoverage
        );
    }

    #[test]
    fn membership_epoch_change_invalidates_old_certificate() {
        let mut scoped = scope();
        scoped.membership_epoch = 8;
        for participant in scoped.participants.values_mut() {
            participant.membership_epoch = 8;
        }
        assert_eq!(
            assess_stable_frontier(&scoped, &certificate()),
            StableFrontierDispositionV1::BlockedMembershipEpoch
        );
    }

    #[test]
    fn fenced_participant_requires_explicit_fence_evidence() {
        let mut scoped = scope();
        scoped.participants.get_mut("node-b").unwrap().state = ParticipantStateV1::Fenced;
        let mut cert = certificate();
        cert.coverage.get_mut("node-b").unwrap().status = FrontierCoverageStatusV1::Fenced;
        assert_eq!(
            assess_stable_frontier(&scoped, &cert),
            StableFrontierDispositionV1::Stable
        );
        cert.coverage.get_mut("node-b").unwrap().status = FrontierCoverageStatusV1::Covered;
        assert_eq!(
            assess_stable_frontier(&scoped, &cert),
            StableFrontierDispositionV1::BlockedFenceEvidence
        );
    }

    #[test]
    fn uncovered_tombstone_cannot_be_reclaimed() {
        let mut cert = certificate();
        cert.covered_frontier_roots.remove("frontier-6");
        let mut tombstones = BTreeMap::new();
        tombstones.insert("tomb-1".into(), tombstone());
        assert_eq!(
            assess_retention_boundary(&boundary(), &scope(), &cert, &tombstones),
            RetentionDispositionV1::BlockedUncoveredTombstone
        );
    }

    #[test]
    fn incomplete_inventory_blocks_pruning() {
        let mut b = boundary();
        b.history_inventory_closed = false;
        let mut tombstones = BTreeMap::new();
        tombstones.insert("tomb-1".into(), tombstone());
        assert_eq!(
            assess_retention_boundary(&b, &scope(), &certificate(), &tombstones),
            RetentionDispositionV1::BlockedIncompleteHistoryInventory
        );
    }

    #[test]
    fn split_view_receipt_cannot_cross_retention_commitments() {
        let b = boundary();
        let mut tombstones = BTreeMap::new();
        tombstones.insert("tomb-1".into(), tombstone());
        let receipt = PruningReceiptV1 {
            pruning_id: "prune-1".into(),
            scope_id: "scope-1".into(),
            semantic_environment_root: "env-1".into(),
            membership_epoch: 7,
            source_snapshot_root: "snapshot-7".into(),
            retention_boundary_commitment: "other-boundary".into(),
            reclaimed_history_roots: b.reclaimable_history_roots.clone(),
            reclaimed_tombstone_ids: b.reclaimable_tombstone_ids.clone(),
            resulting_snapshot_root: "snapshot-8".into(),
            claim_ceiling: "ReferenceModelOnly".into(),
        };
        assert_eq!(
            assess_pruning_receipt(&receipt, &b, &scope(), &certificate(), &tombstones),
            PruningDispositionV1::Conflict
        );
    }

    #[test]
    fn cold_start_missing_history_is_not_normative() {
        let manifest = ColdStartManifestV1 {
            node_id: "node-a".into(),
            incarnation_id: "inc-2".into(),
            semantic_environment_root: "env-1".into(),
            membership_epoch: 7,
            snapshot_root: "snapshot-7".into(),
            snapshot_frontier_root: "frontier-7".into(),
            required_history_roots: BTreeSet::from(["hist-1".into(), "hist-2".into()]),
            retained_tombstone_ids: BTreeSet::from(["tomb-1".into()]),
            reconstruction_profile_id: STABLE_FRONTIER_PROFILE_ID.into(),
            normative_state_root: "state-7".into(),
            manifest_commitment: "manifest-7".into(),
        };
        assert_eq!(
            assess_cold_start_reconstruction(
                &manifest,
                &BTreeSet::from(["hist-1".into()]),
                &BTreeSet::from(["tomb-1".into()]),
            ),
            ReconstructionDispositionV1::BlockedMissingHistory
        );
    }

    #[test]
    fn archived_mirror_missing_tombstone_blocks_reconstruction() {
        let manifest = ColdStartManifestV1 {
            node_id: "node-a".into(),
            incarnation_id: "inc-2".into(),
            semantic_environment_root: "env-1".into(),
            membership_epoch: 7,
            snapshot_root: "snapshot-7".into(),
            snapshot_frontier_root: "frontier-7".into(),
            required_history_roots: BTreeSet::new(),
            retained_tombstone_ids: BTreeSet::from(["tomb-1".into()]),
            reconstruction_profile_id: STABLE_FRONTIER_PROFILE_ID.into(),
            normative_state_root: "state-7".into(),
            manifest_commitment: "manifest-7".into(),
        };
        assert_eq!(
            assess_cold_start_reconstruction(
                &manifest,
                &BTreeSet::new(),
                &BTreeSet::new(),
            ),
            ReconstructionDispositionV1::BlockedMissingTombstone
        );
    }

    #[test]
    fn stale_rejoin_is_fenced_by_membership_or_frontier() {
        let assessment = assess_rejoin(
            &scope(),
            "node-a",
            "inc-2",
            "env-1",
            6,
            "frontier-6",
            "frontier-7",
        );
        assert_eq!(
            assessment.disposition,
            RejoinDispositionV1::BlockedStaleMembership
        );
    }

    #[test]
    fn forgotten_history_cannot_create_claims() {
        let before = ConservedClaimStateV1 {
            authority_claims: BTreeSet::from(["auth-1".into()]),
            capacity_claims: BTreeSet::from(["cap-1".into()]),
            consent_claims: BTreeSet::from(["consent-1".into()]),
        };
        let mut after = before.clone();
        after.capacity_claims.insert("cap-forged".into());
        assert!(!reclamation_cannot_create_claims(&before, &after));
    }

    #[test]
    fn retention_witness_is_claim_bounded() {
        let witness = retention_witness(&scope(), &certificate(), None, &BTreeMap::new());
        assert_eq!(
            witness.disposition,
            RetentionDispositionV1::BlockedWithoutStableFrontier
        );
        assert!(witness.claim_ceiling.contains("no durable-storage"));
    }
}
