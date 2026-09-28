//! Semantic archive continuity and historical evidence integrity reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! An archive can preserve evidence and seed reconstruction. It is never,
//! by itself, a source of present authority, currentness, capacity, consent,
//! actuation, or policy interpretation.

use crate::no_resurrection::SemanticTombstone;
use crate::stable_frontier::{
    assess_cold_start_reconstruction, ColdStartManifestV1, ReconstructionDispositionV1,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const ARCHIVE_CONTINUITY_PROFILE_ID: &str = "INTEGRAL-ARCHIVE-REF-001";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HistoricalClaimClassV1 {
    State,
    Lineage,
    Authority,
    Capacity,
    Consent,
    Actuation,
    PolicyInterpretation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CurrentnessCeilingV1 {
    HistoricalOnly,
    ReconstructionInputOnly,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HistoricalEvidenceProfileV1 {
    pub profile_id: String,
    pub semantic_environment_root: String,
    pub allowed_claim_classes: BTreeSet<HistoricalClaimClassV1>,
    pub currentness_ceiling: CurrentnessCeilingV1,
    pub reconstruction_allowed: bool,
    pub profile_commitment: String,
}

impl HistoricalEvidenceProfileV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.profile_id.is_empty()
            && !self.semantic_environment_root.is_empty()
            && !self.allowed_claim_classes.is_empty()
            && !self.profile_commitment.is_empty()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArchiveCompletenessV1 {
    CompleteForProfile,
    Partial,
    Unknown,
    Unavailable,
    Conflicting,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticArchiveManifestV1 {
    pub archive_id: String,
    pub source_snapshot_root: String,
    pub source_frontier_root: String,
    pub semantic_environment_root: String,
    pub membership_epoch: u64,
    pub profile_id: String,
    pub content_root: String,
    pub inventory_root: String,
    pub retained_tombstone_ids: BTreeSet<String>,
    pub completeness: ArchiveCompletenessV1,
    pub manifest_commitment: String,
}

impl SemanticArchiveManifestV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.archive_id.is_empty()
            && !self.source_snapshot_root.is_empty()
            && !self.source_frontier_root.is_empty()
            && !self.semantic_environment_root.is_empty()
            && !self.profile_id.is_empty()
            && !self.content_root.is_empty()
            && !self.inventory_root.is_empty()
            && !self.manifest_commitment.is_empty()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArchiveContinuityKindV1 {
    CompactedSuccessor,
    MembershipTransition,
    ProfileTransition,
    MembershipAndProfileTransition,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArchiveContinuityCertificateV1 {
    pub certificate_id: String,
    pub predecessor_archive_id: String,
    pub predecessor_frontier_root: String,
    pub successor_archive_id: String,
    pub successor_frontier_root: String,
    pub semantic_environment_root: String,
    pub predecessor_membership_epoch: u64,
    pub successor_membership_epoch: u64,
    pub predecessor_profile_id: String,
    pub successor_profile_id: String,
    pub continuity_kind: ArchiveContinuityKindV1,
    pub membership_transition_root: Option<String>,
    pub profile_transition_root: Option<String>,
    pub certificate_commitment: String,
    pub claim_ceiling: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArchiveContinuityDispositionV1 {
    Accepted,
    BlockedMissingTransitionEvidence,
    BlockedEnvironment,
    BlockedProfile,
    BlockedMembership,
    Conflict,
    InsufficientEvidence,
}

pub fn assess_archive_continuity(
    predecessor: &SemanticArchiveManifestV1,
    successor: &SemanticArchiveManifestV1,
    certificate: &ArchiveContinuityCertificateV1,
) -> ArchiveContinuityDispositionV1 {
    if !predecessor.structurally_valid()
        || !successor.structurally_valid()
        || certificate.certificate_id.is_empty()
        || certificate.certificate_commitment.is_empty()
        || certificate.claim_ceiling.is_empty()
    {
        return ArchiveContinuityDispositionV1::InsufficientEvidence;
    }
    if predecessor.semantic_environment_root != successor.semantic_environment_root
        || certificate.semantic_environment_root != predecessor.semantic_environment_root
    {
        return ArchiveContinuityDispositionV1::BlockedEnvironment;
    }
    if certificate.predecessor_archive_id != predecessor.archive_id
        || certificate.predecessor_frontier_root != predecessor.source_frontier_root
        || certificate.successor_archive_id != successor.archive_id
        || certificate.successor_frontier_root != successor.source_frontier_root
    {
        return ArchiveContinuityDispositionV1::Conflict;
    }
    if certificate.predecessor_membership_epoch != predecessor.membership_epoch
        || certificate.successor_membership_epoch != successor.membership_epoch
    {
        return ArchiveContinuityDispositionV1::BlockedMembership;
    }
    if certificate.predecessor_profile_id != predecessor.profile_id
        || certificate.successor_profile_id != successor.profile_id
    {
        return match certificate.profile_transition_root.as_deref() {
            Some(root) if !root.is_empty() => {
                if certificate.continuity_kind
                    != ArchiveContinuityKindV1::ProfileTransition
                    && certificate.continuity_kind
                        != ArchiveContinuityKindV1::MembershipAndProfileTransition
                {
                    ArchiveContinuityDispositionV1::Conflict
                } else {
                    ArchiveContinuityDispositionV1::Accepted
                }
            }
            _ => ArchiveContinuityDispositionV1::BlockedProfile,
        };
    }

    if predecessor.membership_epoch != successor.membership_epoch {
        if certificate.continuity_kind != ArchiveContinuityKindV1::MembershipTransition {
            return ArchiveContinuityDispositionV1::Conflict;
        }
        if certificate.membership_transition_root.as_deref().is_none() {
            return ArchiveContinuityDispositionV1::BlockedMissingTransitionEvidence;
        }
    } else if certificate.continuity_kind != ArchiveContinuityKindV1::CompactedSuccessor {
        return ArchiveContinuityDispositionV1::Conflict;
    }

    ArchiveContinuityDispositionV1::Accepted
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArchiveUsePurposeV1 {
    HistoricalAnalysis,
    HistoricalAudit,
    ReconstructionInput,
    CurrentAuthorization,
    CurrentActuation,
    CurrentPolicyInterpretation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArchiveUseDispositionV1 {
    UsableHistoricalEvidence,
    UsablePartialHistoricalEvidence,
    UsableReconstructionInput,
    BlockedCurrentAuthority,
    BlockedCurrentActuation,
    BlockedCurrentPolicyInterpretation,
    BlockedIncompleteArchive,
    BlockedEnvironment,
    BlockedProfile,
    BlockedMissingHistory,
    BlockedMissingTombstone,
    BlockedConflictingArchive,
    InsufficientEvidence,
}

fn completeness_allows_historical(completeness: ArchiveCompletenessV1) -> bool {
    matches!(
        completeness,
        ArchiveCompletenessV1::CompleteForProfile | ArchiveCompletenessV1::Partial
    )
}

pub fn assess_archive_use(
    archive: &SemanticArchiveManifestV1,
    profile: &HistoricalEvidenceProfileV1,
    purpose: ArchiveUsePurposeV1,
    semantic_environment_root: &str,
    required_history_roots: &BTreeSet<String>,
    available_history_roots: &BTreeSet<String>,
    required_tombstone_ids: &BTreeSet<String>,
    available_tombstone_ids: &BTreeSet<String>,
) -> ArchiveUseDispositionV1 {
    if !archive.structurally_valid() || !profile.structurally_valid() {
        return ArchiveUseDispositionV1::InsufficientEvidence;
    }
    if archive.semantic_environment_root != semantic_environment_root
        || profile.semantic_environment_root != semantic_environment_root
    {
        return ArchiveUseDispositionV1::BlockedEnvironment;
    }
    if archive.profile_id != profile.profile_id {
        return ArchiveUseDispositionV1::BlockedProfile;
    }

    match purpose {
        ArchiveUsePurposeV1::CurrentAuthorization => {
            ArchiveUseDispositionV1::BlockedCurrentAuthority
        }
        ArchiveUsePurposeV1::CurrentActuation => ArchiveUseDispositionV1::BlockedCurrentActuation,
        ArchiveUsePurposeV1::CurrentPolicyInterpretation => {
            ArchiveUseDispositionV1::BlockedCurrentPolicyInterpretation
        }
        ArchiveUsePurposeV1::HistoricalAnalysis | ArchiveUsePurposeV1::HistoricalAudit => {
            match archive.completeness {
                ArchiveCompletenessV1::CompleteForProfile => {
                    ArchiveUseDispositionV1::UsableHistoricalEvidence
                }
                ArchiveCompletenessV1::Partial => {
                    ArchiveUseDispositionV1::UsablePartialHistoricalEvidence
                }
                ArchiveCompletenessV1::Conflicting => {
                    ArchiveUseDispositionV1::BlockedConflictingArchive
                }
                ArchiveCompletenessV1::Unknown | ArchiveCompletenessV1::Unavailable => {
                    ArchiveUseDispositionV1::BlockedIncompleteArchive
                }
            }
        }
        ArchiveUsePurposeV1::ReconstructionInput => {
            if !profile.reconstruction_allowed {
                return ArchiveUseDispositionV1::BlockedProfile;
            }
            if !matches!(
                profile.currentness_ceiling,
                CurrentnessCeilingV1::ReconstructionInputOnly
                    | CurrentnessCeilingV1::HistoricalOnly
            ) {
                return ArchiveUseDispositionV1::BlockedProfile;
            }
            if archive.completeness != ArchiveCompletenessV1::CompleteForProfile {
                return ArchiveUseDispositionV1::BlockedIncompleteArchive;
            }
            if !required_history_roots.is_subset(available_history_roots) {
                return ArchiveUseDispositionV1::BlockedMissingHistory;
            }
            if !required_tombstone_ids.is_subset(available_tombstone_ids)
                || !required_tombstone_ids.is_subset(&archive.retained_tombstone_ids)
            {
                return ArchiveUseDispositionV1::BlockedMissingTombstone;
            }
            ArchiveUseDispositionV1::UsableReconstructionInput
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HistoricalClaimTemporalScopeV1 {
    HistoricalAtFrontier,
    HistoricalInterval,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HistoricalClaimReceiptV1 {
    pub receipt_id: String,
    pub archive_id: String,
    pub profile_id: String,
    pub semantic_environment_root: String,
    pub claim_class: HistoricalClaimClassV1,
    pub temporal_scope: HistoricalClaimTemporalScopeV1,
    pub source_frontier_root: String,
    pub subject_id: String,
    pub evidence_completeness: ArchiveCompletenessV1,
    pub currentness_ceiling: CurrentnessCeilingV1,
    pub claim_commitment: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HistoricalClaimDispositionV1 {
    AcceptedHistorical,
    BlockedCurrentness,
    BlockedClaimClass,
    BlockedIncompleteEvidence,
    BlockedEnvironment,
    BlockedProfile,
    InsufficientEvidence,
}

pub fn assess_historical_claim(
    receipt: &HistoricalClaimReceiptV1,
    archive: &SemanticArchiveManifestV1,
    profile: &HistoricalEvidenceProfileV1,
    requested_currentness: bool,
) -> HistoricalClaimDispositionV1 {
    if receipt.receipt_id.is_empty()
        || receipt.archive_id.is_empty()
        || receipt.profile_id.is_empty()
        || receipt.semantic_environment_root.is_empty()
        || receipt.source_frontier_root.is_empty()
        || receipt.subject_id.is_empty()
        || receipt.claim_commitment.is_empty()
    {
        return HistoricalClaimDispositionV1::InsufficientEvidence;
    }
    if requested_currentness {
        return HistoricalClaimDispositionV1::BlockedCurrentness;
    }
    if receipt.archive_id != archive.archive_id {
        return HistoricalClaimDispositionV1::BlockedProfile;
    }
    if receipt.profile_id != profile.profile_id
        || archive.profile_id != profile.profile_id
        || receipt.semantic_environment_root != profile.semantic_environment_root
        || archive.semantic_environment_root != profile.semantic_environment_root
    {
        return HistoricalClaimDispositionV1::BlockedEnvironment;
    }
    if !profile.allowed_claim_classes.contains(&receipt.claim_class) {
        return HistoricalClaimDispositionV1::BlockedClaimClass;
    }
    if matches!(
        receipt.evidence_completeness,
        ArchiveCompletenessV1::Unknown
            | ArchiveCompletenessV1::Unavailable
            | ArchiveCompletenessV1::Conflicting
    ) || !completeness_allows_historical(archive.completeness)
    {
        return ArchiveUseDispositionV1::BlockedIncompleteArchive.into();
    }
    HistoricalClaimDispositionV1::AcceptedHistorical
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArchiveConflictClassV1 {
    DivergentContent,
    DivergentFrontier,
    LifecycleConflict,
    ProfileConflict,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArchiveConflictDispositionV1 {
    Contested,
    ResolvedByExplicitTransition,
    InsufficientEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArchiveConflictSetV1 {
    pub conflict_id: String,
    pub archive_ids: BTreeSet<String>,
    pub archive_roots: BTreeSet<String>,
    pub conflict_class: ArchiveConflictClassV1,
    pub disposition: ArchiveConflictDispositionV1,
    pub resolution_receipt_id: Option<String>,
    pub conflict_commitment: String,
}

pub fn assess_archive_conflict(conflict: &ArchiveConflictSetV1) -> ArchiveConflictDispositionV1 {
    if conflict.archive_ids.len() < 2
        || conflict.archive_roots.len() < 2
        || conflict.conflict_commitment.is_empty()
    {
        return ArchiveConflictDispositionV1::InsufficientEvidence;
    }
    if conflict.disposition == ArchiveConflictDispositionV1::ResolvedByExplicitTransition
        && conflict.resolution_receipt_id.as_deref().is_none()
    {
        return ArchiveConflictDispositionV1::InsufficientEvidence;
    }
    conflict.disposition
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArchiveRehydrationDispositionV1 {
    ReadyAsReconstructionInput,
    BlockedCurrentAuthority,
    BlockedArchiveUse,
    BlockedReconstruction,
    BlockedManifestMismatch,
}

pub fn assess_archive_rehydration(
    archive: &SemanticArchiveManifestV1,
    profile: &HistoricalEvidenceProfileV1,
    manifest: &ColdStartManifestV1,
    available_history_roots: &BTreeSet<String>,
    available_tombstone_ids: &BTreeSet<String>,
) -> ArchiveRehydrationDispositionV1 {
    if archive.semantic_environment_root != manifest.semantic_environment_root
        || profile.semantic_environment_root != manifest.semantic_environment_root
        || archive.profile_id != profile.profile_id
    {
        return ArchiveRehydrationDispositionV1::BlockedManifestMismatch;
    }

    let use_disposition = assess_archive_use(
        archive,
        profile,
        ArchiveUsePurposeV1::ReconstructionInput,
        &manifest.semantic_environment_root,
        &manifest.required_history_roots,
        available_history_roots,
        &manifest.retained_tombstone_ids,
        available_tombstone_ids,
    );
    if use_disposition != ArchiveUseDispositionV1::UsableReconstructionInput {
        return ArchiveRehydrationDispositionV1::BlockedArchiveUse;
    }

    match assess_cold_start_reconstruction(
        manifest,
        available_history_roots,
        available_tombstone_ids,
    ) {
        ReconstructionDispositionV1::Ready => {
            ArchiveRehydrationDispositionV1::ReadyAsReconstructionInput
        }
        ReconstructionDispositionV1::BlockedMissingHistory
        | ReconstructionDispositionV1::BlockedMissingTombstone => {
            ArchiveRehydrationDispositionV1::BlockedReconstruction
        }
        ReconstructionDispositionV1::InsufficientEvidence => {
            ArchiveRehydrationDispositionV1::BlockedReconstruction
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArchiveClaimStateV1 {
    pub authority_claims: BTreeSet<String>,
    pub capacity_claims: BTreeSet<String>,
    pub consent_claims: BTreeSet<String>,
}

pub fn archive_use_cannot_create_claims(
    before: &ArchiveClaimStateV1,
    after: &ArchiveClaimStateV1,
) -> bool {
    after.authority_claims.is_subset(&before.authority_claims)
        && after.capacity_claims.is_subset(&before.capacity_claims)
        && after.consent_claims.is_subset(&before.consent_claims)
}

pub fn archive_tombstones_are_present(
    archive: &SemanticArchiveManifestV1,
    required_tombstones: &BTreeSet<String>,
    tombstones: &BTreeMap<String, SemanticTombstone>,
) -> bool {
    required_tombstones.is_subset(&archive.retained_tombstone_ids)
        && required_tombstones.iter().all(|id| tombstones.contains_key(id))
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArchiveWitnessV1 {
    pub archive_id: String,
    pub source_frontier_root: String,
    pub profile_id: String,
    pub disposition: ArchiveUseDispositionV1,
    pub claim_ceiling: String,
}

pub fn archive_witness(
    archive: &SemanticArchiveManifestV1,
    profile: &HistoricalEvidenceProfileV1,
) -> ArchiveWitnessV1 {
    let disposition = if archive.semantic_environment_root == profile.semantic_environment_root
        && archive.profile_id == profile.profile_id
        && archive.structurally_valid()
        && profile.structurally_valid()
    {
        match archive.completeness {
            ArchiveCompletenessV1::CompleteForProfile => {
                ArchiveUseDispositionV1::UsableHistoricalEvidence
            }
            ArchiveCompletenessV1::Partial => {
                ArchiveUseDispositionV1::UsablePartialHistoricalEvidence
            }
            ArchiveCompletenessV1::Conflicting => {
                ArchiveUseDispositionV1::BlockedConflictingArchive
            }
            ArchiveCompletenessV1::Unknown | ArchiveCompletenessV1::Unavailable => {
                ArchiveUseDispositionV1::BlockedIncompleteArchive
            }
        }
    } else {
        ArchiveUseDispositionV1::BlockedProfile
    };

    ArchiveWitnessV1 {
        archive_id: archive.archive_id.clone(),
        source_frontier_root: archive.source_frontier_root.clone(),
        profile_id: profile.profile_id.clone(),
        disposition,
        claim_ceiling: "Historical evidence only; no present authority, currentness, actuation, durable-storage, or production-safety claim.".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> HistoricalEvidenceProfileV1 {
        HistoricalEvidenceProfileV1 {
            profile_id: "profile-1".into(),
            semantic_environment_root: "env-1".into(),
            allowed_claim_classes: BTreeSet::from([
                HistoricalClaimClassV1::State,
                HistoricalClaimClassV1::Lineage,
                HistoricalClaimClassV1::Authority,
                HistoricalClaimClassV1::Capacity,
                HistoricalClaimClassV1::Consent,
            ]),
            currentness_ceiling: CurrentnessCeilingV1::ReconstructionInputOnly,
            reconstruction_allowed: true,
            profile_commitment: "profile-commit".into(),
        }
    }

    fn archive(id: &str, frontier: &str, epoch: u64) -> SemanticArchiveManifestV1 {
        SemanticArchiveManifestV1 {
            archive_id: id.into(),
            source_snapshot_root: format!("snapshot-{id}"),
            source_frontier_root: frontier.into(),
            semantic_environment_root: "env-1".into(),
            membership_epoch: epoch,
            profile_id: "profile-1".into(),
            content_root: format!("content-{id}"),
            inventory_root: format!("inventory-{id}"),
            retained_tombstone_ids: BTreeSet::from(["tomb-1".into()]),
            completeness: ArchiveCompletenessV1::CompleteForProfile,
            manifest_commitment: format!("manifest-{id}"),
        }
    }

    fn cold_manifest() -> ColdStartManifestV1 {
        ColdStartManifestV1 {
            node_id: "node-a".into(),
            incarnation_id: "inc-3".into(),
            semantic_environment_root: "env-1".into(),
            membership_epoch: 7,
            snapshot_root: "snapshot-new".into(),
            snapshot_frontier_root: "frontier-new".into(),
            required_history_roots: BTreeSet::from(["hist-1".into()]),
            retained_tombstone_ids: BTreeSet::from(["tomb-1".into()]),
            reconstruction_profile_id: "profile-1".into(),
            normative_state_root: "state-new".into(),
            manifest_commitment: "cold-manifest".into(),
        }
    }

    #[test]
    fn complete_archive_supports_historical_analysis_but_not_current_authority() {
        let a = archive("a1", "frontier-1", 7);
        let empty = BTreeSet::new();
        assert_eq!(
            assess_archive_use(
                &a,
                &profile(),
                ArchiveUsePurposeV1::HistoricalAnalysis,
                "env-1",
                &empty,
                &empty,
                &empty,
                &empty,
            ),
            ArchiveUseDispositionV1::UsableHistoricalEvidence
        );
        assert_eq!(
            assess_archive_use(
                &a,
                &profile(),
                ArchiveUsePurposeV1::CurrentAuthorization,
                "env-1",
                &empty,
                &empty,
                &empty,
                &empty,
            ),
            ArchiveUseDispositionV1::BlockedCurrentAuthority
        );
    }

    #[test]
    fn partial_archive_is_not_complete_evidence() {
        let mut a = archive("a1", "frontier-1", 7);
        a.completeness = ArchiveCompletenessV1::Partial;
        let empty = BTreeSet::new();
        assert_eq!(
            assess_archive_use(
                &a,
                &profile(),
                ArchiveUsePurposeV1::HistoricalAnalysis,
                "env-1",
                &empty,
                &empty,
                &empty,
                &empty,
            ),
            ArchiveUseDispositionV1::UsablePartialHistoricalEvidence
        );
        assert_eq!(
            assess_archive_use(
                &a,
                &profile(),
                ArchiveUsePurposeV1::ReconstructionInput,
                "env-1",
                &empty,
                &empty,
                &empty,
                &empty,
            ),
            ArchiveUseDispositionV1::BlockedIncompleteArchive
        );
    }

    #[test]
    fn environment_mismatch_blocks_silent_reinterpretation() {
        let a = archive("a1", "frontier-1", 7);
        let empty = BTreeSet::new();
        assert_eq!(
            assess_archive_use(
                &a,
                &profile(),
                ArchiveUsePurposeV1::HistoricalAudit,
                "env-other",
                &empty,
                &empty,
                &empty,
                &empty,
            ),
            ArchiveUseDispositionV1::BlockedEnvironment
        );
    }

    #[test]
    fn membership_change_requires_explicit_continuity() {
        let predecessor = archive("a1", "frontier-1", 7);
        let successor = archive("a2", "frontier-2", 8);
        let certificate = ArchiveContinuityCertificateV1 {
            certificate_id: "cert-1".into(),
            predecessor_archive_id: "a1".into(),
            predecessor_frontier_root: "frontier-1".into(),
            successor_archive_id: "a2".into(),
            successor_frontier_root: "frontier-2".into(),
            semantic_environment_root: "env-1".into(),
            predecessor_membership_epoch: 7,
            successor_membership_epoch: 8,
            predecessor_profile_id: "profile-1".into(),
            successor_profile_id: "profile-1".into(),
            continuity_kind: ArchiveContinuityKindV1::MembershipTransition,
            membership_transition_root: None,
            profile_transition_root: None,
            certificate_commitment: "cert-commit".into(),
            claim_ceiling: "ReferenceModelOnly".into(),
        };
        assert_eq!(
            assess_archive_continuity(&predecessor, &successor, &certificate),
            ArchiveContinuityDispositionV1::BlockedMissingTransitionEvidence
        );
    }

    #[test]
    fn membership_transition_with_explicit_evidence_is_accepted() {
        let predecessor = archive("a1", "frontier-1", 7);
        let successor = archive("a2", "frontier-2", 8);
        let certificate = ArchiveContinuityCertificateV1 {
            certificate_id: "cert-1".into(),
            predecessor_archive_id: "a1".into(),
            predecessor_frontier_root: "frontier-1".into(),
            successor_archive_id: "a2".into(),
            successor_frontier_root: "frontier-2".into(),
            semantic_environment_root: "env-1".into(),
            predecessor_membership_epoch: 7,
            successor_membership_epoch: 8,
            predecessor_profile_id: "profile-1".into(),
            successor_profile_id: "profile-1".into(),
            continuity_kind: ArchiveContinuityKindV1::MembershipTransition,
            membership_transition_root: Some("membership-transition-8".into()),
            profile_transition_root: None,
            certificate_commitment: "cert-commit".into(),
            claim_ceiling: "ReferenceModelOnly".into(),
        };
        assert_eq!(
            assess_archive_continuity(&predecessor, &successor, &certificate),
            ArchiveContinuityDispositionV1::Accepted
        );
    }

    #[test]
    fn profile_change_cannot_be_silent() {
        let predecessor = archive("a1", "frontier-1", 7);
        let mut successor = archive("a2", "frontier-2", 7);
        successor.profile_id = "profile-2".into();
        let certificate = ArchiveContinuityCertificateV1 {
            certificate_id: "cert-1".into(),
            predecessor_archive_id: "a1".into(),
            predecessor_frontier_root: "frontier-1".into(),
            successor_archive_id: "a2".into(),
            successor_frontier_root: "frontier-2".into(),
            semantic_environment_root: "env-1".into(),
            predecessor_membership_epoch: 7,
            successor_membership_epoch: 7,
            predecessor_profile_id: "profile-1".into(),
            successor_profile_id: "profile-2".into(),
            continuity_kind: ArchiveContinuityKindV1::CompactedSuccessor,
            membership_transition_root: None,
            profile_transition_root: None,
            certificate_commitment: "cert-commit".into(),
            claim_ceiling: "ReferenceModelOnly".into(),
        };
        assert_eq!(
            assess_archive_continuity(&predecessor, &successor, &certificate),
            ArchiveContinuityDispositionV1::BlockedProfile
        );
    }

    #[test]
    fn historical_claim_cannot_be_upgraded_to_currentness() {
        let a = archive("a1", "frontier-1", 7);
        let receipt = HistoricalClaimReceiptV1 {
            receipt_id: "claim-1".into(),
            archive_id: "a1".into(),
            profile_id: "profile-1".into(),
            semantic_environment_root: "env-1".into(),
            claim_class: HistoricalClaimClassV1::Authority,
            temporal_scope: HistoricalClaimTemporalScopeV1::HistoricalAtFrontier,
            source_frontier_root: "frontier-1".into(),
            subject_id: "subject-1".into(),
            evidence_completeness: ArchiveCompletenessV1::CompleteForProfile,
            currentness_ceiling: CurrentnessCeilingV1::HistoricalOnly,
            claim_commitment: "claim-commit".into(),
        };
        assert_eq!(
            assess_historical_claim(&receipt, &a, &profile(), true),
            HistoricalClaimDispositionV1::BlockedCurrentness
        );
    }

    #[test]
    fn conflicting_archives_remain_contested_without_resolution() {
        let conflict = ArchiveConflictSetV1 {
            conflict_id: "conflict-1".into(),
            archive_ids: BTreeSet::from(["a1".into(), "a2".into()]),
            archive_roots: BTreeSet::from(["root-1".into(), "root-2".into()]),
            conflict_class: ArchiveConflictClassV1::DivergentContent,
            disposition: ArchiveConflictDispositionV1::Contested,
            resolution_receipt_id: None,
            conflict_commitment: "conflict-commit".into(),
        };
        assert_eq!(
            assess_archive_conflict(&conflict),
            ArchiveConflictDispositionV1::Contested
        );
    }

    #[test]
    fn archive_missing_tombstone_cannot_seed_normative_reconstruction() {
        let a = archive("a1", "frontier-1", 7);
        let empty_history = BTreeSet::from(["hist-1".into()]);
        let empty_tombstones = BTreeSet::new();
        assert_eq!(
            assess_archive_use(
                &a,
                &profile(),
                ArchiveUsePurposeV1::ReconstructionInput,
                "env-1",
                &empty_history,
                &empty_history,
                &BTreeSet::from(["tomb-1".into()]),
                &empty_tombstones,
            ),
            ArchiveUseDispositionV1::BlockedMissingTombstone
        );
    }

    #[test]
    fn complete_archive_can_seed_reconstruction_without_becoming_authority() {
        let a = archive("a1", "frontier-new", 7);
        let history = BTreeSet::from(["hist-1".into()]);
        let tombstones = BTreeSet::from(["tomb-1".into()]);
        assert_eq!(
            assess_archive_rehydration(
                &a,
                &profile(),
                &cold_manifest(),
                &history,
                &tombstones,
            ),
            ArchiveRehydrationDispositionV1::ReadyAsReconstructionInput
        );
        assert_eq!(
            assess_archive_use(
                &a,
                &profile(),
                ArchiveUsePurposeV1::CurrentAuthorization,
                "env-1",
                &BTreeSet::new(),
                &BTreeSet::new(),
                &BTreeSet::new(),
                &BTreeSet::new(),
            ),
            ArchiveUseDispositionV1::BlockedCurrentAuthority
        );
    }

    #[test]
    fn archived_claims_cannot_create_conserved_claims() {
        let before = ArchiveClaimStateV1 {
            authority_claims: BTreeSet::from(["auth-1".into()]),
            capacity_claims: BTreeSet::from(["cap-1".into()]),
            consent_claims: BTreeSet::from(["consent-1".into()]),
        };
        let mut after = before.clone();
        after.authority_claims.insert("auth-forged".into());
        assert!(!archive_use_cannot_create_claims(&before, &after));
    }

    #[test]
    fn tombstone_presence_is_explicitly_checkable() {
        let a = archive("a1", "frontier-1", 7);
        let required = BTreeSet::from(["tomb-1".into()]);
        let mut tombstones = BTreeMap::new();
        assert!(!archive_tombstones_are_present(&a, &required, &tombstones));
        tombstones.insert(
            "tomb-1".into(),
            SemanticTombstone {
                tombstone_id: "tomb-1".into(),
                lineage_id: "lineage-1".into(),
                retired_generation_id: "gen-1".into(),
                retired_creation_event_id: "event-1".into(),
                causal_frontier_root: "frontier-1".into(),
                reason: crate::no_resurrection::TombstoneReason::Revoked,
                provenance_root: "prov-1".into(),
            },
        );
        assert!(archive_tombstones_are_present(&a, &required, &tombstones));
    }

    #[test]
    fn archive_witness_is_claim_bounded() {
        let witness = archive_witness(&archive("a1", "frontier-1", 7), &profile());
        assert!(witness.claim_ceiling.contains("no present authority"));
    }
}
