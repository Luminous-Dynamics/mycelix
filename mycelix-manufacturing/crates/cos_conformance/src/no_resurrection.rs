//! Tombstone, generation, and no-resurrection reference model.
//!
//! Claim ceiling: ReferenceModelOnly.
//! A semantic value may recur without being the same semantic generation.
//! Reactivation is therefore an explicit successor transition, never ID reuse.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const NO_RESURRECTION_PROFILE_ID: &str = "INTEGRAL-LIFECYCLE-REF-001";

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum TombstoneReason {
    Revoked,
    Superseded,
    Retracted,
    Deleted,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticGeneration {
    pub lineage_id: String,
    pub generation_id: String,
    pub state_fingerprint: String,
    pub creation_event_id: String,
    pub semantic_environment_root: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticTombstone {
    pub tombstone_id: String,
    pub lineage_id: String,
    pub retired_generation_id: String,
    pub retired_creation_event_id: String,
    pub causal_frontier_root: String,
    pub reason: TombstoneReason,
    pub provenance_root: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReactivationCandidate {
    pub candidate_id: String,
    pub lineage_id: String,
    pub proposed_generation_id: String,
    pub predecessor_tombstone_id: String,
    pub predecessor_frontier_root: String,
    pub successor_event_id: String,
    pub state_fingerprint: String,
    pub semantic_environment_root: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReactivationDisposition {
    AcceptedSuccessor,
    BlockedResurrection,
    Conflict,
    InsufficientEvidence,
}

pub fn assess_reactivation(
    generation: &SemanticGeneration,
    tombstone: &SemanticTombstone,
    candidate: &ReactivationCandidate,
) -> ReactivationDisposition {
    if generation.lineage_id != tombstone.lineage_id
        || candidate.lineage_id != tombstone.lineage_id
        || generation.semantic_environment_root != candidate.semantic_environment_root
    {
        return ReactivationDisposition::Conflict;
    }
    if generation.generation_id == tombstone.retired_generation_id
        || candidate.proposed_generation_id == tombstone.retired_generation_id
        || candidate.successor_event_id == tombstone.retired_creation_event_id
    {
        return ReactivationDisposition::BlockedResurrection;
    }
    if candidate.predecessor_tombstone_id != tombstone.tombstone_id
        || candidate.predecessor_frontier_root != tombstone.causal_frontier_root
        || candidate.successor_event_id != generation.creation_event_id
    {
        return ReactivationDisposition::InsufficientEvidence;
    }
    ReactivationDisposition::AcceptedSuccessor
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifecycleRecord {
    pub generation: SemanticGeneration,
    pub tombstone: Option<SemanticTombstone>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifecycleLedger {
    pub generations: BTreeMap<String, SemanticGeneration>,
    pub tombstones: BTreeMap<String, SemanticTombstone>,
    pub reactivations: BTreeMap<String, ReactivationCandidate>,
}

impl LifecycleLedger {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert_generation(&mut self, generation: SemanticGeneration) -> bool {
        if self.generations.contains_key(&generation.generation_id) {
            return false;
        }
        self.generations.insert(generation.generation_id.clone(), generation);
        true
    }

    pub fn insert_tombstone(&mut self, tombstone: SemanticTombstone) -> bool {
        if self.tombstones.contains_key(&tombstone.tombstone_id) {
            return false;
        }
        if !self.generations.contains_key(&tombstone.retired_generation_id) {
            return false;
        }
        self.tombstones.insert(tombstone.tombstone_id.clone(), tombstone);
        true
    }

    pub fn record_reactivation(
        &mut self,
        candidate: ReactivationCandidate,
    ) -> ReactivationDisposition {
        if self.reactivations.contains_key(&candidate.candidate_id) {
            return ReactivationDisposition::Conflict;
        }
        let Some(tombstone) = self.tombstones.get(&candidate.predecessor_tombstone_id) else {
            return ReactivationDisposition::InsufficientEvidence;
        };
        let Some(generation) = self.generations.get(&candidate.proposed_generation_id) else {
            return ReactivationDisposition::InsufficientEvidence;
        };
        let disposition = assess_reactivation(generation, tombstone, &candidate);
        if matches!(disposition, ReactivationDisposition::AcceptedSuccessor) {
            self.reactivations.insert(candidate.candidate_id.clone(), candidate);
        }
        disposition
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CacheEntry {
    pub cache_id: String,
    pub lineage_id: String,
    pub generation_id: String,
    pub source_frontier_root: String,
}

pub fn cache_entry_is_normative(
    cache: &CacheEntry,
    active_generation_id: &str,
    tombstones: &BTreeMap<String, SemanticTombstone>,
) -> bool {
    if cache.generation_id != active_generation_id {
        return false;
    }
    !tombstones.values().any(|tombstone| {
        tombstone.lineage_id == cache.lineage_id
            && tombstone.retired_generation_id == cache.generation_id
            && tombstone.causal_frontier_root == cache.source_frontier_root
    })
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CompactionManifest {
    pub snapshot_root: String,
    pub retained_tombstone_ids: BTreeSet<String>,
    pub tombstone_frontier_root: String,
}

pub fn compaction_preserves_tombstones(
    manifest: &CompactionManifest,
    tombstones: &BTreeMap<String, SemanticTombstone>,
) -> bool {
    manifest.retained_tombstone_ids.iter().all(|id| {
        tombstones.get(id).map_or(false, |tombstone| {
            tombstone.causal_frontier_root == manifest.tombstone_frontier_root
        })
    })
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GenerationResources {
    pub generation_id: String,
    pub capacity_claims: BTreeSet<String>,
    pub authority_claims: BTreeSet<String>,
    pub consent_claims: BTreeSet<String>,
}

pub fn successor_resources_may_be_reused(
    retired: &GenerationResources,
    successor: &GenerationResources,
    explicit_reallocation_root: Option<&str>,
) -> bool {
    if explicit_reallocation_root.is_some() {
        return true;
    }
    retired.capacity_claims.is_disjoint(&successor.capacity_claims)
        && retired.authority_claims.is_disjoint(&successor.authority_claims)
        && retired.consent_claims.is_disjoint(&successor.consent_claims)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecycleCompatibility {
    EquivalentHistory,
    CompatibleSuccessors,
    ResurrectionConflict,
    InsufficientEvidence,
}

pub fn reconcile_lifecycle(
    left: &LifecycleRecord,
    right: &LifecycleRecord,
) -> LifecycleCompatibility {
    if left.generation.generation_id == right.generation.generation_id {
        return if left.tombstone == right.tombstone {
            LifecycleCompatibility::EquivalentHistory
        } else {
            LifecycleCompatibility::ResurrectionConflict
        };
    }
    match (&left.tombstone, &right.tombstone) {
        (Some(left_tombstone), Some(right_tombstone))
            if left_tombstone.retired_generation_id == right_tombstone.retired_generation_id
                && left_tombstone.causal_frontier_root == right_tombstone.causal_frontier_root =>
        {
            LifecycleCompatibility::CompatibleSuccessors
        }
        (Some(_), None) | (None, Some(_)) => LifecycleCompatibility::ResurrectionConflict,
        _ => LifecycleCompatibility::InsufficientEvidence,
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResurrectionWitness {
    pub generation_id: String,
    pub tombstone_id: Option<String>,
    pub disposition: ReactivationDisposition,
    pub claim_ceiling: String,
}

pub fn resurrection_witness(
    generation: &SemanticGeneration,
    tombstone: Option<&SemanticTombstone>,
    candidate: Option<&ReactivationCandidate>,
) -> ResurrectionWitness {
    let disposition = match (tombstone, candidate) {
        (Some(tombstone), Some(candidate)) => assess_reactivation(generation, tombstone, candidate),
        _ => ReactivationDisposition::InsufficientEvidence,
    };
    ResurrectionWitness {
        generation_id: generation.generation_id.clone(),
        tombstone_id: tombstone.map(|t| t.tombstone_id.clone()),
        disposition,
        claim_ceiling: "No-resurrection reference semantics only; no storage durability or legal-deletion claim.".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn generation(id: &str, fingerprint: &str) -> SemanticGeneration {
        SemanticGeneration {
            lineage_id: "lineage-1".into(),
            generation_id: id.into(),
            state_fingerprint: fingerprint.into(),
            creation_event_id: format!("event-{id}"),
            semantic_environment_root: "env-1".into(),
        }
    }

    fn tombstone() -> SemanticTombstone {
        SemanticTombstone {
            tombstone_id: "tomb-1".into(),
            lineage_id: "lineage-1".into(),
            retired_generation_id: "gen-1".into(),
            retired_creation_event_id: "event-gen-1".into(),
            causal_frontier_root: "frontier-1".into(),
            reason: TombstoneReason::Revoked,
            provenance_root: "prov-tomb-1".into(),
        }
    }

    fn successor(fingerprint: &str) -> ReactivationCandidate {
        ReactivationCandidate {
            candidate_id: "react-1".into(),
            lineage_id: "lineage-1".into(),
            proposed_generation_id: "gen-2".into(),
            predecessor_tombstone_id: "tomb-1".into(),
            predecessor_frontier_root: "frontier-1".into(),
            successor_event_id: "event-gen-2".into(),
            state_fingerprint: fingerprint.into(),
            semantic_environment_root: "env-1".into(),
        }
    }

    #[test]
    fn identical_old_generation_is_blocked_as_resurrection() {
        let old = generation("gen-1", "same-state");
        let tombstone = tombstone();
        let candidate = ReactivationCandidate {
            candidate_id: "old-replay".into(),
            lineage_id: "lineage-1".into(),
            proposed_generation_id: "gen-1".into(),
            predecessor_tombstone_id: "tomb-1".into(),
            predecessor_frontier_root: "frontier-1".into(),
            successor_event_id: "event-old".into(),
            state_fingerprint: "same-state".into(),
            semantic_environment_root: "env-1".into(),
        };
        assert_eq!(assess_reactivation(&old, &tombstone, &candidate), ReactivationDisposition::BlockedResurrection);
    }

    #[test]
    fn same_visible_value_can_return_only_as_an_explicit_new_generation() {
        let new_generation = generation("gen-2", "same-state");
        let tombstone = tombstone();
        let candidate = successor("same-state");
        assert_eq!(assess_reactivation(&new_generation, &tombstone, &candidate), ReactivationDisposition::AcceptedSuccessor);
    }

    #[test]
    fn old_successor_event_cannot_be_reused() {
        let old_generation = generation("gen-1", "old-state");
        let tombstone = tombstone();
        let candidate = ReactivationCandidate {
            candidate_id: "old-event-replay".into(),
            lineage_id: "lineage-1".into(),
            proposed_generation_id: "gen-1".into(),
            predecessor_tombstone_id: "tomb-1".into(),
            predecessor_frontier_root: "frontier-1".into(),
            successor_event_id: old_generation.creation_event_id.clone(),
            state_fingerprint: "old-state".into(),
            semantic_environment_root: "env-1".into(),
        };
        assert_eq!(
            assess_reactivation(&old_generation, &tombstone, &candidate),
            ReactivationDisposition::BlockedResurrection
        );
    }

    #[test]
    fn explicit_successor_with_new_state_and_frontier_is_accepted() {
        let new_generation = generation("gen-2", "new-state");
        let tombstone = tombstone();
        let candidate = successor("new-state");
        assert_eq!(assess_reactivation(&new_generation, &tombstone, &candidate), ReactivationDisposition::AcceptedSuccessor);
    }

    #[test]
    fn successor_event_must_bind_to_new_generation() {
        let new_generation = generation("gen-2", "new-state");
        let tombstone = tombstone();
        let mut candidate = successor("new-state");
        candidate.successor_event_id = "unrelated-event".into();
        assert_eq!(
            assess_reactivation(&new_generation, &tombstone, &candidate),
            ReactivationDisposition::InsufficientEvidence
        );
    }

    #[test]
    fn wrong_frontier_cannot_authorize_reactivation() {
        let new_generation = generation("gen-2", "new-state");
        let tombstone = tombstone();
        let mut candidate = successor("new-state");
        candidate.predecessor_frontier_root = "wrong-frontier".into();
        assert_eq!(assess_reactivation(&new_generation, &tombstone, &candidate), ReactivationDisposition::InsufficientEvidence);
    }

    #[test]
    fn cache_cannot_reintroduce_retired_generation() {
        let mut tombstones = BTreeMap::new();
        tombstones.insert("tomb-1".into(), tombstone());
        let cache = CacheEntry {
            cache_id: "cache-1".into(),
            lineage_id: "lineage-1".into(),
            generation_id: "gen-1".into(),
            source_frontier_root: "frontier-1".into(),
        };
        assert!(!cache_entry_is_normative(&cache, "gen-1", &tombstones));
    }

    #[test]
    fn compaction_requires_tombstone_frontier_preservation() {
        let mut tombstones = BTreeMap::new();
        tombstones.insert("tomb-1".into(), tombstone());
        let manifest = CompactionManifest {
            snapshot_root: "snapshot-1".into(),
            retained_tombstone_ids: BTreeSet::from(["tomb-1".into()]),
            tombstone_frontier_root: "frontier-1".into(),
        };
        assert!(compaction_preserves_tombstones(&manifest, &tombstones));
        let mut bad = manifest.clone();
        bad.tombstone_frontier_root = "frontier-2".into();
        assert!(!compaction_preserves_tombstones(&bad, &tombstones));
    }

    #[test]
    fn old_authority_and_capacity_do_not_follow_a_successor_without_reallocation() {
        let retired = GenerationResources {
            generation_id: "gen-1".into(),
            capacity_claims: BTreeSet::from(["cap-1".into()]),
            authority_claims: BTreeSet::from(["auth-1".into()]),
            consent_claims: BTreeSet::from(["consent-1".into()]),
        };
        let successor = GenerationResources {
            generation_id: "gen-2".into(),
            capacity_claims: BTreeSet::from(["cap-1".into()]),
            authority_claims: BTreeSet::from(["auth-1".into()]),
            consent_claims: BTreeSet::from(["consent-1".into()]),
        };
        assert!(!successor_resources_may_be_reused(&retired, &successor, None));
        assert!(successor_resources_may_be_reused(&retired, &successor, Some("realloc-1")));
    }

    #[test]
    fn lifecycle_reconciliation_preserves_tombstone_history() {
        let old = generation("gen-1", "old");
        let successor_a = generation("gen-2a", "new-a");
        let successor_b = generation("gen-2b", "new-b");
        let tombstone = tombstone();
        let left = LifecycleRecord {
            generation: successor_a,
            tombstone: Some(tombstone.clone()),
        };
        let right = LifecycleRecord {
            generation: successor_b,
            tombstone: Some(tombstone),
        };
        assert_eq!(reconcile_lifecycle(&left, &right), LifecycleCompatibility::CompatibleSuccessors);
        assert_eq!(old.generation_id, "gen-1");
    }

    #[test]
    fn resurrection_witness_is_claim_bounded() {
        let generation = generation("gen-1", "state");
        let witness = resurrection_witness(&generation, None, None);
        assert_eq!(witness.disposition, ReactivationDisposition::InsufficientEvidence);
        assert!(witness.claim_ceiling.contains("no storage durability"));
    }
}