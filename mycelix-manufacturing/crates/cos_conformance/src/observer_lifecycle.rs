//! Observer and evidence lifecycle continuity reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! D6N makes observer independence explicit. D6O makes that independence
//! temporal: an observer generation, lifecycle status, semantic profile, and
//! dependency snapshot are all scoped to logical frontiers. Historical
//! observations remain immutable evidence after later lifecycle changes, but
//! current qualification must revalidate the observer's present continuity.
//!
//! This module deliberately uses logical frontier sequences/roots rather than
//! wall-clock time. It does not implement revocation infrastructure, key
//! authentication, distributed consensus, or actuation.

use crate::contestable_finality::{
    ExternalObserverRoleV1, ExternalObservedEvidenceV1, ObservationClassificationV1,
    ObservationIndependenceV1,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const OBSERVER_LIFECYCLE_CLAIM_CEILING: &str =
    "Observer/evidence lifecycle reference semantics only; no real-world trust, revocation infrastructure, or authority claim.";

pub const D6O_ELIGIBILITY_RECEIPT_COMMITMENT_DOMAIN: &[u8] =
    b"MYCELIX-INTEGRAL-D6O-ELIGIBILITY-RECEIPT-V1\0";
pub const D6O_GENERATION_COMMITMENT_DOMAIN: &[u8] =
    b"MYCELIX-INTEGRAL-D6O-GENERATION-V1\0";
pub const D6O_TRANSITION_COMMITMENT_DOMAIN: &[u8] =
    b"MYCELIX-INTEGRAL-D6O-TRANSITION-V1\0";
pub const D6O_SNAPSHOT_COMMITMENT_DOMAIN: &[u8] =
    b"MYCELIX-INTEGRAL-D6O-SNAPSHOT-V1\0";
pub const D6O_ROTATION_COMMITMENT_DOMAIN: &[u8] =
    b"MYCELIX-INTEGRAL-D6O-ROTATION-V1\0";
pub const D6O_CONTINUITY_ROOT_DOMAIN: &[u8] =
    b"MYCELIX-INTEGRAL-D6O-CONTINUITY-ROOT-V1\0";
pub const D6O_PROFILE_COMMITMENT_DOMAIN: &[u8] =
    b"MYCELIX-INTEGRAL-D6O-PROFILE-V1\0";


fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn recompute_domain_commitment<T: Serialize>(domain: &[u8], value: &T) -> String {
    let payload = serde_json::to_vec(value).expect("lifecycle commitment serialization must succeed");
    let mut hasher = Sha256::new();
    hasher.update(domain);
    hasher.update(payload);
    format!("{:x}", hasher.finalize())
}

fn is_canonical_sha256_commitment(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ObserverStatusV1 {
    Active,
    Suspended,
    Revoked,
    Retired,
    Superseded,
}

impl ObserverStatusV1 {
    pub fn is_currently_eligible(self) -> bool {
        matches!(self, Self::Active)
    }

    pub fn is_terminal(self) -> bool {
        matches!(self, Self::Revoked | Self::Retired | Self::Superseded)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObserverEvidenceProvenanceV1 {
    Live,
    Archived,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObserverLifecycleUsePurposeV1 {
    HistoricalEvidence,
    CurrentFinalityEligibility,
    CurrentActuationAuthorization,
    ArchiveHistoricalEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EvidenceEligibilityDispositionV1 {
    EligibleCurrent,
    HistoricalOnly,
    BlockedLifecycle,
    BlockedProfile,
    BlockedDependency,
    BlockedContinuity,
    BlockedCurrentness,
    BlockedArchive,
    Contested,
    InsufficientEvidence,
    BlockedAuthorization,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecycleRecordDispositionV1 {
    Recorded,
    BlockedDuplicate,
    Conflict,
    InsufficientEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObserverRotationDispositionV1 {
    Accepted,
    BlockedContinuity,
    BlockedProfile,
    BlockedDependency,
    BlockedLifecycle,
    Conflict,
    InsufficientEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObserverLifecycleProfileV1 {
    pub profile_id: String,
    pub semantic_environment_root: String,
    pub observation_profile_id: String,
    pub allowed_roles: BTreeSet<ExternalObserverRoleV1>,
    pub current_frontier_required: bool,
    pub historical_evidence_allowed: bool,
    pub profile_commitment: String,
    pub claim_ceiling: String,
}

impl ObserverLifecycleProfileV1 {
    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.profile_commitment.clear();
        recompute_domain_commitment(D6O_PROFILE_COMMITMENT_DOMAIN, &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid()
            && (!is_canonical_sha256_commitment(&self.profile_commitment)
                || self.profile_commitment == self.recomputed_commitment())
    }

    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.profile_id)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.observation_profile_id)
            && !self.allowed_roles.is_empty()
            && non_empty(&self.profile_commitment)
            && self.claim_ceiling == OBSERVER_LIFECYCLE_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObserverGenerationV1 {
    pub generation_id: String,
    pub observer_id: String,
    pub generation_sequence: u64,
    pub predecessor_generation_id: Option<String>,
    pub role: ExternalObserverRoleV1,
    pub observation_method: String,
    pub provider_relationship: String,
    pub evidence_root: String,
    pub custody_root: String,
    pub upstream_observer_ids: BTreeSet<String>,
    pub upstream_evidence_roots: BTreeSet<String>,
    pub semantic_environment_root: String,
    pub observation_profile_id: String,
    pub created_frontier_root: String,
    pub created_frontier_sequence: u64,
    pub initial_status: ObserverStatusV1,
    pub generation_commitment: String,
    pub claim_ceiling: String,
}

impl ObserverGenerationV1 {
    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.generation_commitment.clear();
        recompute_domain_commitment(D6O_GENERATION_COMMITMENT_DOMAIN, &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid()
            && (!is_canonical_sha256_commitment(&self.generation_commitment)
                || self.generation_commitment == self.recomputed_commitment())
    }

    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.generation_id)
            && non_empty(&self.observer_id)
            && self.generation_sequence > 0
            && self.predecessor_generation_id.as_deref() != Some(self.generation_id.as_str())
            && non_empty(&self.observation_method)
            && non_empty(&self.provider_relationship)
            && non_empty(&self.evidence_root)
            && non_empty(&self.custody_root)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.observation_profile_id)
            && non_empty(&self.created_frontier_root)
            && non_empty(&self.generation_commitment)
            && self.created_frontier_sequence > 0
            && matches!(self.initial_status, ObserverStatusV1::Active)
            && !self.upstream_observer_ids.contains(&self.observer_id)
            && !self.upstream_evidence_roots.contains(&self.evidence_root)
            && self.claim_ceiling == OBSERVER_LIFECYCLE_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObserverStatusTransitionV1 {
    pub transition_id: String,
    pub observer_id: String,
    pub predecessor_generation_id: String,
    pub successor_generation_id: Option<String>,
    pub from_status: ObserverStatusV1,
    pub to_status: ObserverStatusV1,
    pub effective_frontier_root: String,
    pub effective_frontier_sequence: u64,
    pub semantic_environment_root: String,
    pub observation_profile_id: String,
    pub evidence_root: String,
    pub custody_root: String,
    pub upstream_observer_ids: BTreeSet<String>,
    pub upstream_evidence_roots: BTreeSet<String>,
    pub reason: String,
    pub qualification_transition_id: String,
    pub transition_commitment: String,
    pub claim_ceiling: String,
}

impl ObserverStatusTransitionV1 {
    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.transition_commitment.clear();
        recompute_domain_commitment(D6O_TRANSITION_COMMITMENT_DOMAIN, &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid()
            && (!is_canonical_sha256_commitment(&self.transition_commitment)
                || self.transition_commitment == self.recomputed_commitment())
    }

    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.transition_id)
            && non_empty(&self.observer_id)
            && non_empty(&self.predecessor_generation_id)
            && self.effective_frontier_sequence > 0
            && non_empty(&self.effective_frontier_root)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.observation_profile_id)
            && non_empty(&self.evidence_root)
            && non_empty(&self.custody_root)
            && non_empty(&self.reason)
            && non_empty(&self.qualification_transition_id)
            && non_empty(&self.transition_commitment)
            && self.from_status != self.to_status
            && !matches!(self.to_status, ObserverStatusV1::Active)
            && (matches!(self.to_status, ObserverStatusV1::Superseded)
                == self.successor_generation_id.is_some())
            && self.claim_ceiling == OBSERVER_LIFECYCLE_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceDependencySnapshotV1 {
    pub snapshot_id: String,
    pub observer_generation_id: String,
    pub observation_profile_id: String,
    pub semantic_environment_root: String,
    pub evidence_root: String,
    pub custody_root: String,
    pub upstream_observer_ids: BTreeSet<String>,
    pub upstream_evidence_roots: BTreeSet<String>,
    pub independence: ObservationIndependenceV1,
    pub effective_frontier_root: String,
    pub effective_frontier_sequence: u64,
    pub predecessor_snapshot_id: Option<String>,
    pub snapshot_commitment: String,
    pub claim_ceiling: String,
}

impl EvidenceDependencySnapshotV1 {
    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.snapshot_commitment.clear();
        recompute_domain_commitment(D6O_SNAPSHOT_COMMITMENT_DOMAIN, &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid()
            && (!is_canonical_sha256_commitment(&self.snapshot_commitment)
                || self.snapshot_commitment == self.recomputed_commitment())
    }

    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.snapshot_id)
            && non_empty(&self.observer_generation_id)
            && non_empty(&self.observation_profile_id)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.evidence_root)
            && non_empty(&self.custody_root)
            && non_empty(&self.effective_frontier_root)
            && self.effective_frontier_sequence > 0
            && non_empty(&self.snapshot_commitment)
            && self.predecessor_snapshot_id.as_deref() != Some(self.snapshot_id.as_str())
            && self.claim_ceiling == OBSERVER_LIFECYCLE_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObserverRotationCertificateV1 {
    pub certificate_id: String,
    pub observer_id: String,
    pub predecessor_generation_id: String,
    pub successor_generation_id: String,
    pub predecessor_environment_root: String,
    pub successor_environment_root: String,
    pub predecessor_profile_id: String,
    pub successor_profile_id: String,
    pub predecessor_evidence_root: String,
    pub successor_evidence_root: String,
    pub effective_frontier_root: String,
    pub effective_frontier_sequence: u64,
    pub predecessor_transition_id: String,
    pub qualification_transition_id: String,
    pub continuity_root: String,
    pub certificate_commitment: String,
    pub claim_ceiling: String,
}

impl ObserverRotationCertificateV1 {
    pub fn recomputed_continuity_root(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.continuity_root.clear();
        unsigned.certificate_commitment.clear();
        recompute_domain_commitment(D6O_CONTINUITY_ROOT_DOMAIN, &unsigned)
    }

    pub fn continuity_root_matches(&self) -> bool {
        non_empty(&self.continuity_root)
            && (!is_canonical_sha256_commitment(&self.continuity_root)
                || self.continuity_root == self.recomputed_continuity_root())
    }

    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.certificate_commitment.clear();
        recompute_domain_commitment(D6O_ROTATION_COMMITMENT_DOMAIN, &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid()
            && (!is_canonical_sha256_commitment(&self.certificate_commitment)
                || self.certificate_commitment == self.recomputed_commitment())
    }

    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.certificate_id)
            && non_empty(&self.observer_id)
            && non_empty(&self.predecessor_generation_id)
            && non_empty(&self.successor_generation_id)
            && self.predecessor_generation_id != self.successor_generation_id
            && non_empty(&self.predecessor_environment_root)
            && non_empty(&self.successor_environment_root)
            && non_empty(&self.predecessor_profile_id)
            && non_empty(&self.successor_profile_id)
            && non_empty(&self.predecessor_evidence_root)
            && non_empty(&self.successor_evidence_root)
            && non_empty(&self.effective_frontier_root)
            && self.effective_frontier_sequence > 0
            && non_empty(&self.predecessor_transition_id)
            && non_empty(&self.qualification_transition_id)
            && non_empty(&self.continuity_root)
            && non_empty(&self.certificate_commitment)
            && self.claim_ceiling == OBSERVER_LIFECYCLE_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceEligibilityReceiptV1 {
    pub eligibility_id: String,
    pub observation_id: String,
    pub observer_id: String,
    pub observer_generation_id: String,
    pub observation_profile_id: String,
    pub semantic_environment_root: String,
    pub dependency_snapshot_id: String,
    pub observation_frontier_root: String,
    pub observation_frontier_sequence: u64,
    pub current_frontier_root: String,
    pub current_frontier_sequence: u64,
    pub current_generation_id: Option<String>,
    pub qualification_profile_id: String,
    pub provenance: ObserverEvidenceProvenanceV1,
    pub classification: ObservationClassificationV1,
    pub disposition: EvidenceEligibilityDispositionV1,
    pub lifecycle_transition_ids: BTreeSet<String>,
    pub eligibility_commitment: String,
    pub claim_ceiling: String,
}

impl EvidenceEligibilityReceiptV1 {
    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.eligibility_commitment.clear();
        let payload = serde_json::to_vec(&unsigned).expect("receipt serialization must succeed");
        let mut hasher = Sha256::new();
        hasher.update(D6O_ELIGIBILITY_RECEIPT_COMMITMENT_DOMAIN);
        hasher.update(payload);
        format!("{:x}", hasher.finalize())
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid() && self.eligibility_commitment == self.recomputed_commitment()
    }

    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.eligibility_id)
            && non_empty(&self.observation_id)
            && non_empty(&self.observer_id)
            && non_empty(&self.observer_generation_id)
            && non_empty(&self.observation_profile_id)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.dependency_snapshot_id)
            && non_empty(&self.observation_frontier_root)
            && self.observation_frontier_sequence > 0
            && non_empty(&self.current_frontier_root)
            && self.current_frontier_sequence > 0
            && non_empty(&self.qualification_profile_id)
            && non_empty(&self.eligibility_commitment)
            && self.claim_ceiling == OBSERVER_LIFECYCLE_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObserverLifecycleProposalV1 {
    pub proposal_id: String,
    pub observer_id: String,
    pub predecessor_generation_id: Option<String>,
    pub proposed_generation_id: Option<String>,
    pub proposed_status: ObserverStatusV1,
    pub analysis_commitment: String,
    pub qualified_transition_id: Option<String>,
    pub claim_ceiling: String,
}

impl ObserverLifecycleProposalV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.proposal_id)
            && non_empty(&self.observer_id)
            && non_empty(&self.analysis_commitment)
            && self.claim_ceiling == OBSERVER_LIFECYCLE_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObserverLifecycleLedgerV1 {
    pub generations: BTreeMap<String, ObserverGenerationV1>,
    pub transitions: BTreeMap<String, ObserverStatusTransitionV1>,
    pub dependency_snapshots: BTreeMap<String, EvidenceDependencySnapshotV1>,
    pub rotations: BTreeMap<String, ObserverRotationCertificateV1>,
    pub eligibility_receipts: BTreeMap<String, EvidenceEligibilityReceiptV1>,
}

fn transition_order<'a>(
    transitions: impl Iterator<Item = &'a ObserverStatusTransitionV1>,
) -> Vec<&'a ObserverStatusTransitionV1> {
    let mut ordered: Vec<_> = transitions.collect();
    ordered.sort_by(|a, b| {
        a.effective_frontier_sequence
            .cmp(&b.effective_frontier_sequence)
            .then_with(|| a.transition_id.cmp(&b.transition_id))
    });
    ordered
}

fn dependency_order<'a>(
    snapshots: impl Iterator<Item = &'a EvidenceDependencySnapshotV1>,
) -> Vec<&'a EvidenceDependencySnapshotV1> {
    let mut ordered: Vec<_> = snapshots.collect();
    ordered.sort_by(|a, b| {
        a.effective_frontier_sequence
            .cmp(&b.effective_frontier_sequence)
            .then_with(|| a.snapshot_id.cmp(&b.snapshot_id))
    });
    ordered
}

impl ObserverLifecycleLedgerV1 {
    pub fn record_generation(
        &mut self,
        generation: ObserverGenerationV1,
    ) -> LifecycleRecordDispositionV1 {
        if !generation.commitment_matches() {
            return LifecycleRecordDispositionV1::InsufficientEvidence;
        }

        if let Some(existing) = self.generations.get(&generation.generation_id) {
            return if existing == &generation {
                LifecycleRecordDispositionV1::BlockedDuplicate
            } else {
                LifecycleRecordDispositionV1::Conflict
            };
        }

        if self.generations.values().any(|existing| {
            existing.observer_id == generation.observer_id
                && existing.generation_sequence == generation.generation_sequence
                && existing.generation_id != generation.generation_id
        }) {
            return LifecycleRecordDispositionV1::Conflict;
        }

        self.generations
            .insert(generation.generation_id.clone(), generation);
        LifecycleRecordDispositionV1::Recorded
    }

    fn generation_chain_valid(&self, generation_id: &str) -> bool {
        let Some(generation) = self.generations.get(generation_id) else {
            return false;
        };

        if let Some(predecessor_id) = &generation.predecessor_generation_id {
            let Some(predecessor) = self.generations.get(predecessor_id) else {
                return false;
            };
            if predecessor.observer_id != generation.observer_id
                || predecessor.generation_sequence >= generation.generation_sequence
            {
                return false;
            }
        }

        true
    }

    fn status_at_frontier_internal(
        &self,
        generation_id: &str,
        frontier_sequence: u64,
    ) -> Option<ObserverStatusV1> {
        let generation = self.generations.get(generation_id)?;
        if frontier_sequence < generation.created_frontier_sequence {
            return None;
        }

        let mut status = generation.initial_status;
        let ordered = transition_order(
            self.transitions
                .values()
                .filter(|transition| transition.predecessor_generation_id == generation_id),
        );

        let mut last_sequence = generation.created_frontier_sequence;
        for transition in ordered {
            if transition.effective_frontier_sequence > frontier_sequence {
                break;
            }
            if transition.effective_frontier_sequence <= last_sequence
                || transition.from_status != status
            {
                return None;
            }
            status = transition.to_status;
            last_sequence = transition.effective_frontier_sequence;
        }

        Some(status)
    }

    pub fn status_at_frontier(
        &self,
        generation_id: &str,
        frontier_sequence: u64,
    ) -> Option<ObserverStatusV1> {
        self.status_at_frontier_internal(generation_id, frontier_sequence)
    }

    fn transition_step_is_possible(
        from_status: ObserverStatusV1,
        to_status: ObserverStatusV1,
    ) -> bool {
        matches!(
            (from_status, to_status),
            (ObserverStatusV1::Active, ObserverStatusV1::Suspended)
                | (ObserverStatusV1::Active, ObserverStatusV1::Revoked)
                | (ObserverStatusV1::Active, ObserverStatusV1::Retired)
                | (ObserverStatusV1::Active, ObserverStatusV1::Superseded)
                | (ObserverStatusV1::Suspended, ObserverStatusV1::Revoked)
                | (ObserverStatusV1::Suspended, ObserverStatusV1::Retired)
                | (ObserverStatusV1::Suspended, ObserverStatusV1::Superseded)
        )
    }

    fn validate_transition_chain(&self, generation_id: &str) -> bool {
        let Some(generation) = self.generations.get(generation_id) else {
            return false;
        };
        if !self.generation_chain_valid(generation_id) {
            return false;
        }

        let ordered = transition_order(
            self.transitions
                .values()
                .filter(|transition| transition.predecessor_generation_id == generation_id),
        );
        let mut status = generation.initial_status;
        let mut last_sequence = generation.created_frontier_sequence;

        for transition in ordered {
            if transition.effective_frontier_sequence <= last_sequence
                || transition.from_status != status
                || transition.observer_id != generation.observer_id
                || transition.semantic_environment_root
                    != generation.semantic_environment_root
                || transition.observation_profile_id != generation.observation_profile_id
                || transition.evidence_root != generation.evidence_root
                || transition.custody_root != generation.custody_root
                || transition.upstream_observer_ids != generation.upstream_observer_ids
                || transition.upstream_evidence_roots != generation.upstream_evidence_roots
                || !Self::transition_step_is_possible(transition.from_status, transition.to_status)
            {
                return false;
            }

            if let Some(successor_id) = &transition.successor_generation_id {
                let Some(successor) = self.generations.get(successor_id) else {
                    return false;
                };
                if transition.to_status != ObserverStatusV1::Superseded
                    || successor.predecessor_generation_id.as_deref() != Some(generation_id)
                    || successor.observer_id != generation.observer_id
                    || successor.initial_status != ObserverStatusV1::Active
                    || successor.created_frontier_sequence != transition.effective_frontier_sequence
                    || successor.created_frontier_root != transition.effective_frontier_root
                {
                    return false;
                }
            }

            status = transition.to_status;
            last_sequence = transition.effective_frontier_sequence;
        }

        true
    }

    pub fn record_transition(
        &mut self,
        transition: ObserverStatusTransitionV1,
    ) -> LifecycleRecordDispositionV1 {
        if !transition.commitment_matches()
            || !Self::transition_step_is_possible(transition.from_status, transition.to_status)
        {
            return LifecycleRecordDispositionV1::InsufficientEvidence;
        }

        let Some(generation) = self.generations.get(&transition.predecessor_generation_id) else {
            return LifecycleRecordDispositionV1::InsufficientEvidence;
        };

        if generation.observer_id != transition.observer_id
            || transition.effective_frontier_sequence <= generation.created_frontier_sequence
            || transition.semantic_environment_root != generation.semantic_environment_root
            || transition.observation_profile_id != generation.observation_profile_id
            || transition.evidence_root != generation.evidence_root
            || transition.custody_root != generation.custody_root
            || transition.upstream_observer_ids != generation.upstream_observer_ids
            || transition.upstream_evidence_roots != generation.upstream_evidence_roots
        {
            return LifecycleRecordDispositionV1::Conflict;
        }

        if let Some(existing) = self.transitions.get(&transition.transition_id) {
            return if existing == &transition {
                LifecycleRecordDispositionV1::BlockedDuplicate
            } else {
                LifecycleRecordDispositionV1::Conflict
            };
        }

        if self.transitions.values().any(|existing| {
            existing.predecessor_generation_id == transition.predecessor_generation_id
                && existing.effective_frontier_sequence
                    == transition.effective_frontier_sequence
                && existing != &transition
        }) {
            return LifecycleRecordDispositionV1::Conflict;
        }

        if let Some(successor_id) = transition.successor_generation_id.as_deref() {
            let Some(successor) = self.generations.get(successor_id) else {
                return LifecycleRecordDispositionV1::InsufficientEvidence;
            };
            if successor.predecessor_generation_id.as_deref()
                != Some(transition.predecessor_generation_id.as_str())
                || successor.created_frontier_sequence != transition.effective_frontier_sequence
                || successor.created_frontier_root != transition.effective_frontier_root
            {
                return LifecycleRecordDispositionV1::Conflict;
            }
        }

        let mut tentative = self.clone();
        tentative
            .transitions
            .insert(transition.transition_id.clone(), transition.clone());

        let chain_valid = tentative.validate_transition_chain(&transition.predecessor_generation_id);
        if !chain_valid {
            let ordered = transition_order(
                tentative
                    .transitions
                    .values()
                    .filter(|candidate| {
                        candidate.predecessor_generation_id
                            == transition.predecessor_generation_id
                    }),
            );
            let has_missing_predecessor = ordered
                .first()
                .is_some_and(|first| first.from_status != generation.initial_status);

            if !has_missing_predecessor {
                return LifecycleRecordDispositionV1::Conflict;
            }
        }

        self.transitions
            .insert(transition.transition_id.clone(), transition);
        LifecycleRecordDispositionV1::Recorded
    }

    pub fn record_dependency_snapshot(
        &mut self,
        snapshot: EvidenceDependencySnapshotV1,
    ) -> LifecycleRecordDispositionV1 {
        if !snapshot.commitment_matches() {
            return LifecycleRecordDispositionV1::InsufficientEvidence;
        }

        let Some(generation) = self.generations.get(&snapshot.observer_generation_id) else {
            return LifecycleRecordDispositionV1::InsufficientEvidence;
        };

        if snapshot.observation_profile_id != generation.observation_profile_id
            || snapshot.semantic_environment_root != generation.semantic_environment_root
            || snapshot.effective_frontier_sequence < generation.created_frontier_sequence
        {
            return LifecycleRecordDispositionV1::Conflict;
        }

        if let Some(existing) = self.dependency_snapshots.get(&snapshot.snapshot_id) {
            return if existing == &snapshot {
                LifecycleRecordDispositionV1::BlockedDuplicate
            } else {
                LifecycleRecordDispositionV1::Conflict
            };
        }

        if self.dependency_snapshots.values().any(|existing| {
            existing.observer_generation_id == snapshot.observer_generation_id
                && existing.effective_frontier_sequence == snapshot.effective_frontier_sequence
                && existing != &snapshot
        }) {
            return LifecycleRecordDispositionV1::Conflict;
        }

        if let Some(predecessor_id) = snapshot.predecessor_snapshot_id.as_deref() {
            if let Some(predecessor) = self.dependency_snapshots.get(predecessor_id) {
                if predecessor.observer_generation_id != snapshot.observer_generation_id
                    || predecessor.effective_frontier_sequence
                        >= snapshot.effective_frontier_sequence
                {
                    return LifecycleRecordDispositionV1::Conflict;
                }
            }
        } else if snapshot.effective_frontier_sequence != generation.created_frontier_sequence {
            return LifecycleRecordDispositionV1::InsufficientEvidence;
        }

        self.dependency_snapshots
            .insert(snapshot.snapshot_id.clone(), snapshot);
        LifecycleRecordDispositionV1::Recorded
    }

    fn dependency_chain_complete(&self, snapshot_id: &str) -> bool {
        let mut current_id = snapshot_id.to_owned();
        let mut visited = BTreeSet::new();

        loop {
            if !visited.insert(current_id.clone()) {
                return false;
            }
            let Some(snapshot) = self.dependency_snapshots.get(&current_id) else {
                return false;
            };
            let Some(predecessor_id) = snapshot.predecessor_snapshot_id.as_deref() else {
                let Some(generation) = self.generations.get(&snapshot.observer_generation_id) else {
                    return false;
                };
                return snapshot.effective_frontier_sequence == generation.created_frontier_sequence;
            };
            let Some(predecessor) = self.dependency_snapshots.get(predecessor_id) else {
                return false;
            };
            if predecessor.observer_generation_id != snapshot.observer_generation_id
                || predecessor.effective_frontier_sequence
                    >= snapshot.effective_frontier_sequence
            {
                return false;
            }
            current_id = predecessor_id.to_owned();
        }
    }

    pub fn dependency_snapshot_at(
        &self,
        generation_id: &str,
        frontier_sequence: u64,
    ) -> Option<&EvidenceDependencySnapshotV1> {
        dependency_order(
            self.dependency_snapshots
                .values()
                .filter(|snapshot| {
                    snapshot.observer_generation_id == generation_id
                        && snapshot.effective_frontier_sequence <= frontier_sequence
                        && self.dependency_chain_complete(&snapshot.snapshot_id)
                }),
        )
        .into_iter()
        .last()
    }

    pub fn record_rotation(
        &mut self,
        certificate: ObserverRotationCertificateV1,
    ) -> ObserverRotationDispositionV1 {
        if !certificate.commitment_matches() || !certificate.continuity_root_matches() {
            return ObserverRotationDispositionV1::InsufficientEvidence;
        }

        let Some(predecessor) = self.generations.get(&certificate.predecessor_generation_id) else {
            return ObserverRotationDispositionV1::InsufficientEvidence;
        };
        let Some(successor) = self.generations.get(&certificate.successor_generation_id) else {
            return ObserverRotationDispositionV1::InsufficientEvidence;
        };
        let Some(transition) = self.transitions.get(&certificate.predecessor_transition_id) else {
            return ObserverRotationDispositionV1::InsufficientEvidence;
        };

        if self.rotations.values().any(|existing| {
            existing.predecessor_generation_id == certificate.predecessor_generation_id
                && existing.successor_generation_id != certificate.successor_generation_id
        }) {
            return ObserverRotationDispositionV1::Conflict;
        }

        let disposition =
            assess_observer_rotation(predecessor, successor, transition, &certificate);
        if !matches!(disposition, ObserverRotationDispositionV1::Accepted) {
            return disposition;
        }

        match self.rotations.get(&certificate.certificate_id) {
            Some(existing) if existing == &certificate => {
                ObserverRotationDispositionV1::Conflict
            }
            Some(_) => ObserverRotationDispositionV1::Conflict,
            None => {
                self.rotations
                    .insert(certificate.certificate_id.clone(), certificate);
                ObserverRotationDispositionV1::Accepted
            }
        }
    }

    pub fn current_continuous_generation_id(
        &self,
        observer_id: &str,
        current_frontier_sequence: u64,
    ) -> Option<String> {
        let mut roots: Vec<_> = self
            .generations
            .values()
            .filter(|generation| {
                generation.observer_id == observer_id
                    && generation.predecessor_generation_id.is_none()
                    && generation.created_frontier_sequence <= current_frontier_sequence
            })
            .collect();

        if roots.len() != 1 {
            return None;
        }

        let mut current_id = roots.remove(0).generation_id.clone();
        let mut visited = BTreeSet::new();

        loop {
            if !visited.insert(current_id.clone()) {
                return None;
            }

            let Some(generation) = self.generations.get(&current_id) else {
                return None;
            };
            if !self.validate_transition_chain(&current_id) {
                return None;
            }

            let successors: Vec<_> = self
                .rotations
                .values()
                .filter(|rotation| {
                    rotation.observer_id == observer_id
                        && rotation.predecessor_generation_id == current_id
                        && rotation.effective_frontier_sequence <= current_frontier_sequence
                })
                .collect();

            if successors.len() > 1 {
                return None;
            }

            if let Some(rotation) = successors.first() {
                let Some(successor) = self.generations.get(&rotation.successor_generation_id) else {
                    return None;
                };
                if successor.created_frontier_sequence > current_frontier_sequence {
                    return None;
                }
                current_id = successor.generation_id.clone();
                continue;
            }

            let status = self.status_at_frontier_internal(&generation.generation_id, current_frontier_sequence)?;
            if !status.is_currently_eligible() {
                return None;
            }

            return Some(current_id);
        }
    }

    pub fn record_eligibility_receipt(
        &mut self,
        receipt: EvidenceEligibilityReceiptV1,
    ) -> LifecycleRecordDispositionV1 {
        if !receipt.commitment_matches() {
            return LifecycleRecordDispositionV1::InsufficientEvidence;
        }
        match self.eligibility_receipts.get(&receipt.eligibility_id) {
            Some(existing) if existing == &receipt => LifecycleRecordDispositionV1::BlockedDuplicate,
            Some(_) => LifecycleRecordDispositionV1::Conflict,
            None => {
                self.eligibility_receipts
                    .insert(receipt.eligibility_id.clone(), receipt);
                LifecycleRecordDispositionV1::Recorded
            }
        }
    }
}

pub fn assess_observer_rotation(
    predecessor: &ObserverGenerationV1,
    successor: &ObserverGenerationV1,
    transition: &ObserverStatusTransitionV1,
    certificate: &ObserverRotationCertificateV1,
) -> ObserverRotationDispositionV1 {
    if !predecessor.commitment_matches()
        || !successor.commitment_matches()
        || !transition.commitment_matches()
        || !certificate.commitment_matches()
        || !certificate.continuity_root_matches()
    {
        return ObserverRotationDispositionV1::InsufficientEvidence;
    }

    if predecessor.observer_id != successor.observer_id
        || transition.observer_id != predecessor.observer_id
        || certificate.observer_id != predecessor.observer_id
    {
        return ObserverRotationDispositionV1::Conflict;
    }

    if successor.predecessor_generation_id.as_deref()
        != Some(predecessor.generation_id.as_str())
        || successor.generation_sequence <= predecessor.generation_sequence
        || transition.predecessor_generation_id != predecessor.generation_id
        || transition.successor_generation_id.as_deref()
            != Some(successor.generation_id.as_str())
        || transition.to_status != ObserverStatusV1::Superseded
        || transition.from_status != ObserverStatusV1::Active
    {
        return ObserverRotationDispositionV1::BlockedContinuity;
    }

    if transition.effective_frontier_sequence != successor.created_frontier_sequence
        || transition.effective_frontier_root != successor.created_frontier_root
        || certificate.effective_frontier_sequence != transition.effective_frontier_sequence
        || certificate.effective_frontier_root != transition.effective_frontier_root
    {
        return ObserverRotationDispositionV1::BlockedContinuity;
    }

    if certificate.predecessor_generation_id != predecessor.generation_id
        || certificate.successor_generation_id != successor.generation_id
        || certificate.predecessor_transition_id != transition.transition_id
        || certificate.predecessor_environment_root != predecessor.semantic_environment_root
        || certificate.successor_environment_root != successor.semantic_environment_root
        || certificate.predecessor_profile_id != predecessor.observation_profile_id
        || certificate.successor_profile_id != successor.observation_profile_id
        || certificate.predecessor_evidence_root != predecessor.evidence_root
        || certificate.successor_evidence_root != successor.evidence_root
        || certificate.qualification_transition_id != transition.qualification_transition_id
    {
        return ObserverRotationDispositionV1::Conflict;
    }

    ObserverRotationDispositionV1::Accepted
}

fn classification_disposition(
    classification: ObservationClassificationV1,
) -> Option<EvidenceEligibilityDispositionV1> {
    match classification {
        ObservationClassificationV1::ContradictoryIndependent
        | ObservationClassificationV1::ContradictoryDependent => {
            Some(EvidenceEligibilityDispositionV1::Contested)
        }
        ObservationClassificationV1::Stale => {
            Some(EvidenceEligibilityDispositionV1::BlockedCurrentness)
        }
        ObservationClassificationV1::Superseded => {
            Some(EvidenceEligibilityDispositionV1::BlockedLifecycle)
        }
        ObservationClassificationV1::Incomparable
        | ObservationClassificationV1::InsufficientEvidence => {
            Some(EvidenceEligibilityDispositionV1::InsufficientEvidence)
        }
        ObservationClassificationV1::CorroboratingIndependent
        | ObservationClassificationV1::CorroboratingDependent => None,
    }
}

fn generation_semantics_match(
    evidence: &ExternalObservedEvidenceV1,
    generation: &ObserverGenerationV1,
) -> bool {
    evidence.observer_id == generation.observer_id
        && evidence.observer.observer_id == generation.observer_id
        && evidence.observer.role == generation.role
        && evidence.observer.observation_method == generation.observation_method
        && evidence.observer.provider_relationship == generation.provider_relationship
        && evidence.observer.evidence_root == generation.evidence_root
        && evidence.observer.custody_root == generation.custody_root
        && evidence.observer.upstream_observer_ids == generation.upstream_observer_ids
        && evidence.observer.upstream_evidence_roots == generation.upstream_evidence_roots
        && evidence.observer.semantic_environment_root == generation.semantic_environment_root
        && evidence.observer.observation_profile_id == generation.observation_profile_id
}

fn dependency_snapshot_matches_evidence(
    evidence: &ExternalObservedEvidenceV1,
    snapshot: &EvidenceDependencySnapshotV1,
) -> bool {
    evidence.observer.evidence_root == snapshot.evidence_root
        && evidence.observer.custody_root == snapshot.custody_root
        && evidence.observer.upstream_observer_ids == snapshot.upstream_observer_ids
        && evidence.observer.upstream_evidence_roots == snapshot.upstream_evidence_roots
        && evidence.observer.semantic_environment_root == snapshot.semantic_environment_root
        && evidence.observer.observation_profile_id == snapshot.observation_profile_id
}

pub fn assess_evidence_eligibility(
    evidence: &ExternalObservedEvidenceV1,
    generation: &ObserverGenerationV1,
    snapshot: &EvidenceDependencySnapshotV1,
    profile: &ObserverLifecycleProfileV1,
    ledger: &ObserverLifecycleLedgerV1,
    classification: ObservationClassificationV1,
    provenance: ObserverEvidenceProvenanceV1,
    observation_frontier_root: &str,
    observation_frontier_sequence: u64,
    current_frontier_root: &str,
    current_frontier_sequence: u64,
    purpose: ObserverLifecycleUsePurposeV1,
) -> EvidenceEligibilityDispositionV1 {
    if matches!(
        purpose,
        ObserverLifecycleUsePurposeV1::CurrentActuationAuthorization
    ) {
        return EvidenceEligibilityDispositionV1::BlockedAuthorization;
    }

    if !evidence.structurally_valid()
        || !evidence.observation.commitment_matches()
        || !generation.structurally_valid()
        || !snapshot.structurally_valid()
        || !profile.commitment_matches()
        || observation_frontier_sequence == 0
        || current_frontier_sequence == 0
        || !non_empty(observation_frontier_root)
        || !non_empty(current_frontier_root)
    {
        return EvidenceEligibilityDispositionV1::InsufficientEvidence;
    }

    if let Some(disposition) = classification_disposition(classification) {
        return disposition;
    }

    if evidence.observation.observed_frontier_root != observation_frontier_root
        || observation_frontier_sequence > current_frontier_sequence
    {
        return EvidenceEligibilityDispositionV1::BlockedCurrentness;
    }

    if evidence.observer_id != generation.observer_id
        || evidence.observer.observer_id != generation.observer_id
    {
        return EvidenceEligibilityDispositionV1::BlockedLifecycle;
    }

    if !generation_semantics_match(evidence, generation) {
        return EvidenceEligibilityDispositionV1::BlockedProfile;
    }

    if !profile.allowed_roles.contains(&generation.role)
        || profile.observation_profile_id != generation.observation_profile_id
        || profile.semantic_environment_root != generation.semantic_environment_root
    {
        return EvidenceEligibilityDispositionV1::BlockedProfile;
    }

    if snapshot.observer_generation_id != generation.generation_id
        || snapshot.observation_profile_id != generation.observation_profile_id
        || snapshot.semantic_environment_root != generation.semantic_environment_root
        || !dependency_snapshot_matches_evidence(evidence, snapshot)
        || snapshot.effective_frontier_sequence > observation_frontier_sequence
    {
        return EvidenceEligibilityDispositionV1::BlockedDependency;
    }

    if snapshot.independence != ObservationIndependenceV1::DeclaredIndependent {
        return EvidenceEligibilityDispositionV1::BlockedDependency;
    }

    let Some(snapshot_at_observation) =
        ledger.dependency_snapshot_at(&generation.generation_id, observation_frontier_sequence)
    else {
        return EvidenceEligibilityDispositionV1::BlockedDependency;
    };
    if snapshot_at_observation.snapshot_id != snapshot.snapshot_id {
        return EvidenceEligibilityDispositionV1::BlockedDependency;
    }

    let Some(status_at_observation) =
        ledger.status_at_frontier(&generation.generation_id, observation_frontier_sequence)
    else {
        return EvidenceEligibilityDispositionV1::BlockedLifecycle;
    };
    if !status_at_observation.is_currently_eligible() {
        return EvidenceEligibilityDispositionV1::BlockedLifecycle;
    }

    if matches!(
        provenance,
        ObserverEvidenceProvenanceV1::Archived
    ) {
        if matches!(
            purpose,
            ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility
        ) {
            return EvidenceEligibilityDispositionV1::BlockedArchive;
        }
        return if profile.historical_evidence_allowed {
            EvidenceEligibilityDispositionV1::HistoricalOnly
        } else {
            EvidenceEligibilityDispositionV1::BlockedArchive
        };
    }

    if matches!(
        purpose,
        ObserverLifecycleUsePurposeV1::HistoricalEvidence
        | ObserverLifecycleUsePurposeV1::ArchiveHistoricalEvidence
    ) {
        return if profile.historical_evidence_allowed {
            EvidenceEligibilityDispositionV1::HistoricalOnly
        } else {
            EvidenceEligibilityDispositionV1::InsufficientEvidence
        };
    }

    if profile.current_frontier_required && current_frontier_root.is_empty() {
        return EvidenceEligibilityDispositionV1::BlockedCurrentness;
    }

    let Some(current_generation_id) =
        ledger.current_continuous_generation_id(&generation.observer_id, current_frontier_sequence)
    else {
        return EvidenceEligibilityDispositionV1::BlockedContinuity;
    };

    if current_generation_id != generation.generation_id {
        return EvidenceEligibilityDispositionV1::BlockedContinuity;
    }

    let Some(current_status) =
        ledger.status_at_frontier(&generation.generation_id, current_frontier_sequence)
    else {
        return EvidenceEligibilityDispositionV1::BlockedLifecycle;
    };
    if !current_status.is_currently_eligible() {
        return EvidenceEligibilityDispositionV1::BlockedLifecycle;
    }

    let Some(current_snapshot) =
        ledger.dependency_snapshot_at(&generation.generation_id, current_frontier_sequence)
    else {
        return EvidenceEligibilityDispositionV1::BlockedDependency;
    };
    if current_snapshot.snapshot_id != snapshot.snapshot_id {
        return EvidenceEligibilityDispositionV1::BlockedDependency;
    }

    EvidenceEligibilityDispositionV1::EligibleCurrent
}

pub fn eligibility_receipt_matches(
    receipt: &EvidenceEligibilityReceiptV1,
    evidence: &ExternalObservedEvidenceV1,
    generation: &ObserverGenerationV1,
    snapshot: &EvidenceDependencySnapshotV1,
    profile: &ObserverLifecycleProfileV1,
) -> bool {
    receipt.structurally_valid()
        && evidence.structurally_valid()
        && evidence.observation.commitment_matches()
        && generation.structurally_valid()
        && snapshot.structurally_valid()
        && profile.commitment_matches()
        && receipt.observation_id == evidence.observation.observation_id
        && receipt.observer_id == evidence.observer_id
        && receipt.observer_id == generation.observer_id
        && evidence.observer_id == generation.observer_id
        && receipt.observer_generation_id == generation.generation_id
        && snapshot.observer_generation_id == generation.generation_id
        && receipt.observation_profile_id == generation.observation_profile_id
        && snapshot.observation_profile_id == generation.observation_profile_id
        && receipt.semantic_environment_root == generation.semantic_environment_root
        && snapshot.semantic_environment_root == generation.semantic_environment_root
        && receipt.dependency_snapshot_id == snapshot.snapshot_id
        && receipt.observation_frontier_root == evidence.observation.observed_frontier_root
        && receipt.qualification_profile_id == profile.profile_id
}

fn expected_eligibility_transition_ids(
    generation_id: &str,
    current_frontier_sequence: u64,
    ledger: &ObserverLifecycleLedgerV1,
) -> BTreeSet<String> {
    ledger
        .transitions
        .values()
        .filter(|transition| {
            transition.predecessor_generation_id == generation_id
                && transition.effective_frontier_sequence <= current_frontier_sequence
        })
        .map(|transition| transition.transition_id.clone())
        .collect()
}

/// Reconstruct the D6O qualified boundary from the authoritative lifecycle ledger.
///
/// This verifies the receipt against the same eligibility procedure that produced it,
/// rather than treating receipt fields as authority. The caller supplies the authoritative
/// generation, dependency snapshot, profile, ledger, and observed evidence.
pub fn verify_eligibility_receipt_provenance(
    receipt: &EvidenceEligibilityReceiptV1,
    evidence: &ExternalObservedEvidenceV1,
    generation: &ObserverGenerationV1,
    snapshot: &EvidenceDependencySnapshotV1,
    profile: &ObserverLifecycleProfileV1,
    ledger: &ObserverLifecycleLedgerV1,
) -> bool {
    if !receipt.commitment_matches() {
        return false;
    }
    if !eligibility_receipt_matches(receipt, evidence, generation, snapshot, profile) {
        return false;
    }
    if receipt.current_generation_id.as_deref() != Some(generation.generation_id.as_str()) {
        return false;
    }
    if !generation.commitment_matches() || !snapshot.commitment_matches() || !profile.commitment_matches() {
        return false;
    }
    if !ledger.transitions.values().all(ObserverStatusTransitionV1::commitment_matches) {
        return false;
    }
    if receipt.lifecycle_transition_ids
        != expected_eligibility_transition_ids(
            &generation.generation_id,
            receipt.current_frontier_sequence,
            ledger,
        )
    {
        return false;
    }

    let expected = assess_evidence_eligibility(
        evidence,
        generation,
        snapshot,
        profile,
        ledger,
        receipt.classification,
        receipt.provenance,
        receipt.observation_frontier_root.as_str(),
        receipt.observation_frontier_sequence,
        receipt.current_frontier_root.as_str(),
        receipt.current_frontier_sequence,
        ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
    );

    receipt.disposition == expected
        && receipt.disposition == EvidenceEligibilityDispositionV1::EligibleCurrent
        && ledger.current_continuous_generation_id(
            &generation.observer_id,
            receipt.current_frontier_sequence,
        ) == receipt.current_generation_id
        && ledger
            .dependency_snapshot_at(
                &generation.generation_id,
                receipt.current_frontier_sequence,
            )
            .is_some_and(|current| current.snapshot_id == snapshot.snapshot_id)
}

pub fn lifecycle_proposal_is_authoritative(
    proposal: &ObserverLifecycleProposalV1,
) -> bool {
    let _ = proposal;
    false
}

pub fn lifecycle_transition_can_authorize_actuation(
    transition: &ObserverStatusTransitionV1,
) -> bool {
    let _ = transition;
    false
}

pub fn lifecycle_transition_can_mint_authority_capacity_or_consent(
    transition: &ObserverStatusTransitionV1,
) -> bool {
    let _ = transition;
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::effect_finality::{
        ExternalEffectObservationV1, ExternalObservedStateV1,
        ExternalObservationSourceV1,
    };

    fn profile() -> ObserverLifecycleProfileV1 {
        let allowed_roles = [
            ExternalObserverRoleV1::IndependentObserver,
            ExternalObserverRoleV1::SettlementAuthority,
        ]
        .into_iter()
        .collect::<BTreeSet<_>>();

        ObserverLifecycleProfileV1 {
            profile_id: "life-profile-1".into(),
            semantic_environment_root: "env-1".into(),
            observation_profile_id: "obs-profile-1".into(),
            allowed_roles,
            current_frontier_required: true,
            historical_evidence_allowed: true,
            profile_commitment: "life-profile-commitment".into(),
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        }
    }

    fn generation(id: &str, sequence: u64, predecessor: Option<&str>) -> ObserverGenerationV1 {
        ObserverGenerationV1 {
            generation_id: id.into(),
            observer_id: "observer-A".into(),
            generation_sequence: sequence,
            predecessor_generation_id: predecessor.map(str::to_owned),
            role: ExternalObserverRoleV1::IndependentObserver,
            observation_method: "independent-state-read".into(),
            provider_relationship: "external".into(),
            evidence_root: format!("evidence-{id}"),
            custody_root: format!("custody-{id}"),
            upstream_observer_ids: BTreeSet::new(),
            upstream_evidence_roots: BTreeSet::new(),
            semantic_environment_root: "env-1".into(),
            observation_profile_id: "obs-profile-1".into(),
            created_frontier_root: format!("frontier-{sequence}"),
            created_frontier_sequence: sequence,
            initial_status: ObserverStatusV1::Active,
            generation_commitment: format!("generation:{id}"),
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        }
    }

    fn snapshot(
        id: &str,
        generation_id: &str,
        sequence: u64,
        predecessor: Option<&str>,
        evidence_root: &str,
        custody_root: &str,
        independence: ObservationIndependenceV1,
    ) -> EvidenceDependencySnapshotV1 {
        EvidenceDependencySnapshotV1 {
            snapshot_id: id.into(),
            observer_generation_id: generation_id.into(),
            observation_profile_id: "obs-profile-1".into(),
            semantic_environment_root: "env-1".into(),
            evidence_root: evidence_root.into(),
            custody_root: custody_root.into(),
            upstream_observer_ids: BTreeSet::new(),
            upstream_evidence_roots: BTreeSet::new(),
            independence,
            effective_frontier_root: format!("frontier-{sequence}"),
            effective_frontier_sequence: sequence,
            predecessor_snapshot_id: predecessor.map(str::to_owned),
            snapshot_commitment: format!("snapshot:{id}"),
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        }
    }

    fn evidence(
        id: &str,
        generation: &ObserverGenerationV1,
        frontier_root: &str,
        state: ExternalObservedStateV1,
    ) -> ExternalObservedEvidenceV1 {
        let mut observation = ExternalEffectObservationV1 {
            observation_id: id.into(),
            effect_id: "effect-1".into(),
            effect_lineage_id: "lineage-1".into(),
            lifecycle_generation_id: "generation-1".into(),
            route_id: "route-1".into(),
            provider_id: "provider-1".into(),
            provider_operation_id: "operation-1".into(),
            provider_profile_root: "provider-profile-1".into(),
            provider_outcome_id: format!("outcome-{id}"),
            request_commitment: "request-1".into(),
            idempotency_key: "idem-1".into(),
            semantic_environment_root: "env-1".into(),
            observed_frontier_root: frontier_root.into(),
            observed_state: state,
            source: ExternalObservationSourceV1::IndependentObserver,
            evidence_root: generation.evidence_root.clone(),
            observation_commitment: String::new(),
            claim_ceiling: crate::effect_finality::EXTERNAL_FINALITY_CLAIM_CEILING.into(),
        };
        observation.observation_commitment = observation.recomputed_commitment();

        ExternalObservedEvidenceV1 {
            observation,
            observer_id: generation.observer_id.clone(),
            observer: crate::contestable_finality::ExternalObserverProfileV1 {
                observer_id: generation.observer_id.clone(),
                role: generation.role,
                observation_method: generation.observation_method.clone(),
                provider_relationship: generation.provider_relationship.clone(),
                evidence_root: generation.evidence_root.clone(),
                custody_root: generation.custody_root.clone(),
                upstream_observer_ids: generation.upstream_observer_ids.clone(),
                upstream_evidence_roots: generation.upstream_evidence_roots.clone(),
                semantic_environment_root: generation.semantic_environment_root.clone(),
                observation_profile_id: generation.observation_profile_id.clone(),
                independence: ObservationIndependenceV1::DeclaredIndependent,
                independence_commitment: format!("indep:{}", generation.observer_id),
                claim_ceiling: crate::contestable_finality::CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
            },
        }
    }

    fn active_ledger() -> (ObserverLifecycleLedgerV1, ObserverGenerationV1, EvidenceDependencySnapshotV1) {
        let generation = generation("observer-A-g1", 1, None);
        let snapshot = snapshot(
            "snapshot-1",
            &generation.generation_id,
            1,
            None,
            &generation.evidence_root,
            &generation.custody_root,
            ObservationIndependenceV1::DeclaredIndependent,
        );
        let mut ledger = ObserverLifecycleLedgerV1::default();
        assert_eq!(
            ledger.record_generation(generation.clone()),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_dependency_snapshot(snapshot.clone()),
            LifecycleRecordDispositionV1::Recorded
        );
        (ledger, generation, snapshot)
    }

    fn eligibility(
        ledger: &ObserverLifecycleLedgerV1,
        evidence: &ExternalObservedEvidenceV1,
        generation: &ObserverGenerationV1,
        snapshot: &EvidenceDependencySnapshotV1,
        purpose: ObserverLifecycleUsePurposeV1,
        classification: ObservationClassificationV1,
        observation_sequence: u64,
        current_sequence: u64,
    ) -> EvidenceEligibilityDispositionV1 {
        assess_evidence_eligibility(
            evidence,
            generation,
            snapshot,
            &profile(),
            ledger,
            classification,
            ObserverEvidenceProvenanceV1::Live,
            &evidence.observation.observed_frontier_root,
            observation_sequence,
            &format!("frontier-{current_sequence}"),
            current_sequence,
            purpose,
        )
    }

    fn transition(
        generation: &ObserverGenerationV1,
        transition_id: &str,
        to_status: ObserverStatusV1,
        sequence: u64,
        successor_generation_id: Option<&str>,
    ) -> ObserverStatusTransitionV1 {
        ObserverStatusTransitionV1 {
            transition_id: transition_id.into(),
            observer_id: generation.observer_id.clone(),
            predecessor_generation_id: generation.generation_id.clone(),
            successor_generation_id: successor_generation_id.map(str::to_owned),
            from_status: ObserverStatusV1::Active,
            to_status,
            effective_frontier_root: format!("frontier-{sequence}"),
            effective_frontier_sequence: sequence,
            semantic_environment_root: generation.semantic_environment_root.clone(),
            observation_profile_id: generation.observation_profile_id.clone(),
            evidence_root: generation.evidence_root.clone(),
            custody_root: generation.custody_root.clone(),
            upstream_observer_ids: generation.upstream_observer_ids.clone(),
            upstream_evidence_roots: generation.upstream_evidence_roots.clone(),
            reason: format!("transition-{transition_id}"),
            qualification_transition_id: format!("qualification-{transition_id}"),
            transition_commitment: format!("transition:{transition_id}"),
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        }
    }

    fn rotation_certificate(
        predecessor: &ObserverGenerationV1,
        successor: &ObserverGenerationV1,
        transition: &ObserverStatusTransitionV1,
    ) -> ObserverRotationCertificateV1 {
        ObserverRotationCertificateV1 {
            certificate_id: "rotation-1".into(),
            observer_id: predecessor.observer_id.clone(),
            predecessor_generation_id: predecessor.generation_id.clone(),
            successor_generation_id: successor.generation_id.clone(),
            predecessor_environment_root: predecessor.semantic_environment_root.clone(),
            successor_environment_root: successor.semantic_environment_root.clone(),
            predecessor_profile_id: predecessor.observation_profile_id.clone(),
            successor_profile_id: successor.observation_profile_id.clone(),
            predecessor_evidence_root: predecessor.evidence_root.clone(),
            successor_evidence_root: successor.evidence_root.clone(),
            effective_frontier_root: transition.effective_frontier_root.clone(),
            effective_frontier_sequence: transition.effective_frontier_sequence,
            predecessor_transition_id: transition.transition_id.clone(),
            qualification_transition_id: transition.qualification_transition_id.clone(),
            continuity_root: "continuity-root-1".into(),
            certificate_commitment: "rotation-commitment-1".into(),
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        }
    }

    #[test]
    fn stale_d6m_observation_commitment_is_rejected_at_d6o_boundary() {
        let (ledger, generation, snapshot) = active_ledger();
        let valid = evidence(
            "obs-d6m-integrity",
            &generation,
            "frontier-1",
            ExternalObservedStateV1::Applied,
        );
        assert!(valid.observation.commitment_matches());

        let mut stale = valid.clone();
        stale.observation.observed_state = ExternalObservedStateV1::Reversed;
        assert!(!stale.observation.commitment_matches());

        assert_eq!(
            eligibility(
                &ledger,
                &stale,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                1,
            ),
            EvidenceEligibilityDispositionV1::InsufficientEvidence
        );
    }

    #[test]
    fn active_observer_qualifies() {
        let (ledger, generation, snapshot) = active_ledger();
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                1
            ),
            EvidenceEligibilityDispositionV1::EligibleCurrent
        );
    }

    #[test]
    fn revoked_before_observation_frontier_blocks_current_qualification() {
        let (mut ledger, generation, snapshot) = active_ledger();
        let t = transition(&generation, "revoke-1", ObserverStatusV1::Revoked, 2, None);
        assert_eq!(
            ledger.record_transition(t),
            LifecycleRecordDispositionV1::Recorded
        );
        let e = evidence("obs-2", &generation, "frontier-2", ExternalObservedStateV1::Applied);
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                2,
                2
            ),
            EvidenceEligibilityDispositionV1::BlockedLifecycle
        );
    }

    #[test]
    fn suspended_observer_blocks_new_current_independent_evidence() {
        let (mut ledger, generation, snapshot) = active_ledger();
        let t = transition(&generation, "suspend-1", ObserverStatusV1::Suspended, 2, None);
        assert_eq!(
            ledger.record_transition(t),
            LifecycleRecordDispositionV1::Recorded
        );
        let e = evidence("obs-2", &generation, "frontier-2", ExternalObservedStateV1::Applied);
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                2,
                2
            ),
            EvidenceEligibilityDispositionV1::BlockedLifecycle
        );
    }

    #[test]
    fn old_evidence_remains_historical_after_revocation() {
        let (mut ledger, generation, snapshot) = active_ledger();
        let t = transition(&generation, "revoke-1", ObserverStatusV1::Revoked, 3, None);
        assert_eq!(
            ledger.record_transition(t),
            LifecycleRecordDispositionV1::Recorded
        );
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::HistoricalEvidence,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                3
            ),
            EvidenceEligibilityDispositionV1::HistoricalOnly
        );
    }

    #[test]
    fn old_evidence_cannot_be_reused_for_new_current_qualification() {
        let (mut ledger, generation, snapshot) = active_ledger();
        let t = transition(&generation, "revoke-1", ObserverStatusV1::Revoked, 3, None);
        assert_eq!(
            ledger.record_transition(t),
            LifecycleRecordDispositionV1::Recorded
        );
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                3
            ),
            EvidenceEligibilityDispositionV1::BlockedContinuity
        );
    }

    #[test]
    fn rotation_with_explicit_continuity_is_accepted() {
        let predecessor = generation("observer-A-g1", 1, None);
        let successor = generation("observer-A-g2", 2, Some("observer-A-g1"));
        let transition = transition(
            &predecessor,
            "rotate-1",
            ObserverStatusV1::Superseded,
            2,
            Some(&successor.generation_id),
        );
        assert_eq!(
            assess_observer_rotation(
                &predecessor,
                &successor,
                &transition,
                &rotation_certificate(&predecessor, &successor, &transition)
            ),
            ObserverRotationDispositionV1::Accepted
        );
    }

    #[test]
    fn rotation_without_continuity_evidence_is_blocked() {
        let predecessor = generation("observer-A-g1", 1, None);
        let successor = generation("observer-A-g2", 2, Some("observer-A-g1"));
        let transition = transition(
            &predecessor,
            "rotate-1",
            ObserverStatusV1::Superseded,
            2,
            Some(&successor.generation_id),
        );
        let mut certificate = rotation_certificate(&predecessor, &successor, &transition);
        certificate.continuity_root.clear();
        assert_eq!(
            assess_observer_rotation(&predecessor, &successor, &transition, &certificate),
            ObserverRotationDispositionV1::InsufficientEvidence
        );
    }

    #[test]
    fn same_observer_id_new_generation_is_not_silently_continuous() {
        let predecessor = generation("observer-A-g1", 1, None);
        let mut successor = generation("observer-A-g2", 2, Some("observer-A-g1"));
        successor.observation_profile_id = "obs-profile-2".into();
        let mut ledger = ObserverLifecycleLedgerV1::default();
        assert_eq!(
            ledger.record_generation(predecessor.clone()),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_generation(successor.clone()),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.current_continuous_generation_id("observer-A", 2),
            Some("observer-A-g1".into())
        );

        let successor_snapshot = snapshot(
            "snapshot-2",
            &successor.generation_id,
            2,
            None,
            &successor.evidence_root,
            &successor.custody_root,
            ObservationIndependenceV1::DeclaredIndependent,
        );
        ledger.record_dependency_snapshot(successor_snapshot.clone());
        let e = evidence(
            "obs-successor",
            &successor,
            "frontier-2",
            ExternalObservedStateV1::Applied,
        );
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &successor,
                &successor_snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                2,
                2
            ),
            EvidenceEligibilityDispositionV1::BlockedContinuity
        );
    }

    #[test]
    fn dependency_root_change_downgrades_future_independence() {
        let (mut ledger, generation, base_snapshot) = active_ledger();
        let changed = snapshot(
            "snapshot-2",
            &generation.generation_id,
            2,
            Some("snapshot-1"),
            "new-evidence-root",
            &generation.custody_root,
            ObservationIndependenceV1::DeclaredIndependent,
        );
        assert_eq!(
            ledger.record_dependency_snapshot(changed),
            LifecycleRecordDispositionV1::Recorded
        );
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &base_snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                2
            ),
            EvidenceEligibilityDispositionV1::BlockedDependency
        );
    }

    #[test]
    fn semantic_environment_change_requires_new_generation() {
        let mut successor = generation("observer-A-g2", 2, Some("observer-A-g1"));
        successor.semantic_environment_root = "env-2".into();
        let predecessor = generation("observer-A-g1", 1, None);
        let transition = transition(
            &predecessor,
            "rotate-1",
            ObserverStatusV1::Superseded,
            2,
            Some(&successor.generation_id),
        );
        let mut certificate = rotation_certificate(&predecessor, &successor, &transition);
        certificate.successor_environment_root = "wrong-env".into();
        assert_eq!(
            assess_observer_rotation(&predecessor, &successor, &transition, &certificate),
            ObserverRotationDispositionV1::Conflict
        );
    }

    #[test]
    fn rotation_cannot_reuse_generation_sequence() {
        let predecessor = generation("observer-A-g1", 2, None);
        let successor = generation("observer-A-g2", 2, Some("observer-A-g1"));
        let transition = transition(
            &predecessor,
            "rotate-1",
            ObserverStatusV1::Superseded,
            3,
            Some(&successor.generation_id),
        );
        assert_eq!(
            assess_observer_rotation(
                &predecessor,
                &successor,
                &transition,
                &rotation_certificate(&predecessor, &successor, &transition)
            ),
            ObserverRotationDispositionV1::BlockedContinuity
        );
    }

    #[test]
    fn observation_profile_change_requires_explicit_continuity() {
        let predecessor = generation("observer-A-g1", 1, None);
        let mut successor = generation("observer-A-g2", 2, Some("observer-A-g1"));
        successor.observation_profile_id = "obs-profile-2".into();
        let transition = transition(
            &predecessor,
            "rotate-1",
            ObserverStatusV1::Superseded,
            2,
            Some(&successor.generation_id),
        );
        let certificate = rotation_certificate(&predecessor, &successor, &transition);
        assert_eq!(
            assess_observer_rotation(&predecessor, &successor, &transition, &certificate),
            ObserverRotationDispositionV1::Accepted
        );
    }

    #[test]
    fn provider_role_change_blocks_without_explicit_transition() {
        let (ledger, generation, snapshot) = active_ledger();
        let mut e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        e.observer.role = ExternalObserverRoleV1::Provider;
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                1
            ),
            EvidenceEligibilityDispositionV1::BlockedProfile
        );
    }

    #[test]
    fn archive_mirror_does_not_become_independent() {
        let (ledger, generation, snapshot) = active_ledger();
        let mut archive_generation = generation.clone();
        archive_generation.role = ExternalObserverRoleV1::ArchiveMirror;
        let mut e = evidence(
            "obs-archive",
            &archive_generation,
            "frontier-1",
            ExternalObservedStateV1::Applied,
        );
        e.observer.independence = ObservationIndependenceV1::DeclaredIndependent;
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &archive_generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                1
            ),
            EvidenceEligibilityDispositionV1::BlockedProfile
        );
    }

    #[test]
    fn reactivated_generation_cannot_resurrect_old_evidence() {
        let predecessor = generation("observer-A-g1", 1, None);
        let successor = generation("observer-A-g2", 2, Some("observer-A-g1"));
        let mut ledger = ObserverLifecycleLedgerV1::default();
        ledger.record_generation(predecessor.clone());
        ledger.record_generation(successor.clone());
        let transition = transition(
            &predecessor,
            "rotate-1",
            ObserverStatusV1::Superseded,
            2,
            Some(&successor.generation_id),
        );
        ledger.record_transition(transition.clone());
        ledger.record_rotation(rotation_certificate(
            &predecessor,
            &successor,
            &transition,
        ));
        let old_snapshot = snapshot(
            "snapshot-1",
            &predecessor.generation_id,
            1,
            None,
            &predecessor.evidence_root,
            &predecessor.custody_root,
            ObservationIndependenceV1::DeclaredIndependent,
        );
        ledger.record_dependency_snapshot(old_snapshot.clone());
        let e = evidence(
            "obs-old",
            &predecessor,
            "frontier-1",
            ExternalObservedStateV1::Applied,
        );
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &predecessor,
                &old_snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                2
            ),
            EvidenceEligibilityDispositionV1::BlockedContinuity
        );
    }

    #[test]
    fn eligibility_receipt_commitment_binds_semantic_fields() {
        let (_ledger, generation, snapshot) = active_ledger();
        let mut receipt = EvidenceEligibilityReceiptV1 {
            eligibility_id: "eligibility-1".into(),
            observation_id: "obs-1".into(),
            observer_id: generation.observer_id.clone(),
            observer_generation_id: generation.generation_id.clone(),
            observation_profile_id: generation.observation_profile_id.clone(),
            semantic_environment_root: generation.semantic_environment_root.clone(),
            dependency_snapshot_id: snapshot.snapshot_id.clone(),
            observation_frontier_root: "frontier-1".into(),
            observation_frontier_sequence: 1,
            current_frontier_root: "frontier-1".into(),
            current_frontier_sequence: 1,
            current_generation_id: Some(generation.generation_id.clone()),
            qualification_profile_id: profile().profile_id.clone(),
            provenance: ObserverEvidenceProvenanceV1::Live,
            classification: ObservationClassificationV1::CorroboratingIndependent,
            disposition: EvidenceEligibilityDispositionV1::EligibleCurrent,
            lifecycle_transition_ids: BTreeSet::new(),
            eligibility_commitment: String::new(),
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        };
        receipt.eligibility_commitment = receipt.recomputed_commitment();
        assert!(receipt.commitment_matches());

        let mut forged = receipt.clone();
        forged.current_frontier_root = "frontier-2".into();
        assert!(!forged.commitment_matches());

        let mut forged = receipt.clone();
        forged.lifecycle_transition_ids.insert("transition-forged".into());
        assert!(!forged.commitment_matches());

        let mut forged = receipt.clone();
        forged.classification = ObservationClassificationV1::ContradictoryIndependent;
        assert!(!forged.commitment_matches());
    }

    #[test]
    fn eligibility_receipt_recording_requires_canonical_commitment() {
        let (mut ledger, generation, snapshot) = active_ledger();
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        let mut receipt = EvidenceEligibilityReceiptV1 {
            eligibility_id: "eligibility-record-1".into(),
            observation_id: e.observation.observation_id.clone(),
            observer_id: generation.observer_id.clone(),
            observer_generation_id: generation.generation_id.clone(),
            observation_profile_id: generation.observation_profile_id.clone(),
            semantic_environment_root: generation.semantic_environment_root.clone(),
            dependency_snapshot_id: snapshot.snapshot_id.clone(),
            observation_frontier_root: "frontier-1".into(),
            observation_frontier_sequence: 1,
            current_frontier_root: "frontier-1".into(),
            current_frontier_sequence: 1,
            current_generation_id: Some(generation.generation_id.clone()),
            qualification_profile_id: profile().profile_id,
            provenance: ObserverEvidenceProvenanceV1::Live,
            classification: ObservationClassificationV1::CorroboratingIndependent,
            disposition: EvidenceEligibilityDispositionV1::EligibleCurrent,
            lifecycle_transition_ids: BTreeSet::new(),
            eligibility_commitment: String::new(),
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        };

        assert_eq!(
            ledger.record_eligibility_receipt(receipt.clone()),
            LifecycleRecordDispositionV1::InsufficientEvidence
        );

        receipt.eligibility_commitment = receipt.recomputed_commitment();
        assert_eq!(
            ledger.record_eligibility_receipt(receipt),
            LifecycleRecordDispositionV1::Recorded
        );
    }

    #[test]
    fn authoritative_receipt_reconstruction_rejects_recomputed_transition_provenance_substitution() {
        let (mut ledger, generation, snapshot) = active_ledger();
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        let mut receipt = EvidenceEligibilityReceiptV1 {
            eligibility_id: "eligibility-transition-1".into(),
            observation_id: e.observation.observation_id.clone(),
            observer_id: generation.observer_id.clone(),
            observer_generation_id: generation.generation_id.clone(),
            observation_profile_id: generation.observation_profile_id.clone(),
            semantic_environment_root: generation.semantic_environment_root.clone(),
            dependency_snapshot_id: snapshot.snapshot_id.clone(),
            observation_frontier_root: "frontier-1".into(),
            observation_frontier_sequence: 1,
            current_frontier_root: "frontier-1".into(),
            current_frontier_sequence: 1,
            current_generation_id: Some(generation.generation_id.clone()),
            qualification_profile_id: profile().profile_id,
            provenance: ObserverEvidenceProvenanceV1::Live,
            classification: ObservationClassificationV1::CorroboratingIndependent,
            disposition: EvidenceEligibilityDispositionV1::EligibleCurrent,
            lifecycle_transition_ids: BTreeSet::new(),
            eligibility_commitment: String::new(),
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        };
        receipt.eligibility_commitment = receipt.recomputed_commitment();

        let mut forged = receipt.clone();
        forged.lifecycle_transition_ids.insert("forged-transition".into());
        forged.eligibility_commitment = forged.recomputed_commitment();

        assert!(!verify_eligibility_receipt_provenance(
            &forged, &e, &generation, &snapshot, &profile(), &ledger
        ));

        let transition = transition(&generation, "post-receipt-suspend", ObserverStatusV1::Suspended, 2, None);
        assert_eq!(
            ledger.record_transition(transition),
            LifecycleRecordDispositionV1::Recorded
        );
        assert!(!verify_eligibility_receipt_provenance(
            &receipt, &e, &generation, &snapshot, &profile(), &ledger
        ));
    }

    #[test]
    fn canonical_transition_commitment_is_required_at_ledger_boundary() {
        let generation = generation("observer-A-g1", 1, None);
        let mut transition = transition(
            &generation,
            "transition-ledger-canonical",
            ObserverStatusV1::Suspended,
            2,
            None,
        );
        transition.transition_commitment = transition.recomputed_commitment();

        let mut ledger = ObserverLifecycleLedgerV1::default();
        assert_eq!(
            ledger.record_generation(generation.clone()),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_transition(transition.clone()),
            LifecycleRecordDispositionV1::Recorded
        );

        transition.reason = "forged-after-signing".into();
        assert_eq!(
            ledger.record_transition(transition),
            LifecycleRecordDispositionV1::Conflict
        );
    }

    #[test]
    fn canonical_profile_commitment_binds_semantic_policy() {
        let mut canonical = profile();
        canonical.profile_commitment = canonical.recomputed_commitment();
        assert!(canonical.commitment_matches());
        canonical.allowed_roles.remove(&ExternalObserverRoleV1::SettlementAuthority);
        assert!(!canonical.commitment_matches());
    }

    #[test]
    fn canonical_continuity_root_binds_rotation_semantics() {
        let predecessor = generation("observer-A-g1", 1, None);
        let successor = generation("observer-A-g2", 2, Some(&predecessor.generation_id));
        let transition = transition(
            &predecessor,
            "rotate-continuity-canonical",
            ObserverStatusV1::Superseded,
            2,
            Some(&successor.generation_id),
        );
        let mut certificate = rotation_certificate(&predecessor, &successor, &transition);
        certificate.continuity_root = certificate.recomputed_continuity_root();
        assert!(certificate.continuity_root_matches());
        certificate.successor_profile_id = "profile-substituted".into();
        assert!(!certificate.continuity_root_matches());
    }

    #[test]
    fn canonical_d6o_commitments_bind_authoritative_lifecycle_objects() {
        let base_generation = generation("observer-A-g1", 1, None);
        let mut canonical_generation = base_generation.clone();
        canonical_generation.generation_commitment = canonical_generation.recomputed_commitment();
        assert!(canonical_generation.commitment_matches());
        canonical_generation.evidence_root = "evidence-substituted".into();
        assert!(!canonical_generation.commitment_matches());

        let mut suspend_transition = transition(
            &base_generation,
            "suspend-canonical",
            ObserverStatusV1::Suspended,
            2,
            None,
        );
        suspend_transition.transition_commitment = suspend_transition.recomputed_commitment();
        assert!(suspend_transition.commitment_matches());
        suspend_transition.reason = "forged-reason".into();
        assert!(!suspend_transition.commitment_matches());

        let mut snapshot = snapshot(
            "snapshot-canonical",
            &base_generation.generation_id,
            1,
            None,
            &base_generation.evidence_root,
            &base_generation.custody_root,
            ObservationIndependenceV1::DeclaredIndependent,
        );
        snapshot.snapshot_commitment = snapshot.recomputed_commitment();
        assert!(snapshot.commitment_matches());
        snapshot.custody_root = "custody-substituted".into();
        assert!(!snapshot.commitment_matches());

        let mut successor = generation("observer-A-g2", 2, Some(&base_generation.generation_id));
        let mut rotation_transition = transition(
            &base_generation,
            "rotate-canonical",
            ObserverStatusV1::Superseded,
            2,
            Some(&successor.generation_id),
        );
        rotation_transition.transition_commitment = rotation_transition.recomputed_commitment();
        let mut certificate = rotation_certificate(&base_generation, &successor, &rotation_transition);
        certificate.certificate_commitment = certificate.recomputed_commitment();
        assert!(certificate.commitment_matches());
        certificate.successor_environment_root = "env-substituted".into();
        assert!(!certificate.commitment_matches());

        successor.generation_commitment = successor.recomputed_commitment();
    }

    #[test]
    fn eligibility_receipt_commitment_domain_is_versioned_and_nul_terminated() {
        assert!(D6O_ELIGIBILITY_RECEIPT_COMMITMENT_DOMAIN.ends_with(&[0]));
        assert_eq!(
            D6O_ELIGIBILITY_RECEIPT_COMMITMENT_DOMAIN,
            b"MYCELIX-INTEGRAL-D6O-ELIGIBILITY-RECEIPT-V1\0"
        );
    }

    #[test]
    fn eligibility_receipt_binds_exact_observer_generation() {
        let (ledger, generation, snapshot) = active_ledger();
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        let receipt = EvidenceEligibilityReceiptV1 {
            eligibility_id: "eligibility-1".into(),
            observation_id: "obs-1".into(),
            observer_id: "observer-A".into(),
            observer_generation_id: generation.generation_id.clone(),
            observation_profile_id: generation.observation_profile_id.clone(),
            semantic_environment_root: generation.semantic_environment_root.clone(),
            dependency_snapshot_id: snapshot.snapshot_id.clone(),
            observation_frontier_root: "frontier-1".into(),
            observation_frontier_sequence: 1,
            current_frontier_root: "frontier-1".into(),
            current_frontier_sequence: 1,
            current_generation_id: Some(generation.generation_id.clone()),
            qualification_profile_id: profile().profile_id.clone(),
            provenance: ObserverEvidenceProvenanceV1::Live,
            classification: ObservationClassificationV1::CorroboratingIndependent,
            disposition: EvidenceEligibilityDispositionV1::EligibleCurrent,
            lifecycle_transition_ids: BTreeSet::new(),
            eligibility_commitment: "eligibility-commitment".into(),
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        };
        assert!(eligibility_receipt_matches(
            &receipt,
            &e,
            &generation,
            &snapshot,
            &profile()
        ));
        let mut forged_observer = receipt.clone();
        forged_observer.observer_id = "observer-B".into();
        assert!(!eligibility_receipt_matches(
            &forged_observer,
            &e,
            &generation,
            &snapshot,
            &profile()
        ));

        let mut forged_snapshot_generation = snapshot.clone();
        forged_snapshot_generation.observer_generation_id = "observer-A-g2".into();
        assert!(!eligibility_receipt_matches(
            &receipt,
            &e,
            &generation,
            &forged_snapshot_generation,
            &profile()
        ));

        let mut forged = receipt.clone();
        forged.observer_generation_id = "observer-A-g2".into();
        assert!(!eligibility_receipt_matches(
            &forged,
            &e,
            &generation,
            &snapshot,
            &profile()
        ));
        let _ = ledger;
    }

    #[test]
    fn stale_lifecycle_status_at_current_frontier_is_rejected() {
        let (mut ledger, generation, snapshot) = active_ledger();
        let t = transition(&generation, "suspend-1", ObserverStatusV1::Suspended, 2, None);
        ledger.record_transition(t);
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                2
            ),
            EvidenceEligibilityDispositionV1::BlockedContinuity
        );
    }

    #[test]
    fn currentness_revalidation_after_revocation_is_conservative() {
        let (mut ledger, generation, snapshot) = active_ledger();
        let t = transition(&generation, "revoke-1", ObserverStatusV1::Revoked, 2, None);
        ledger.record_transition(t);
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::HistoricalEvidence,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                2
            ),
            EvidenceEligibilityDispositionV1::HistoricalOnly
        );
        assert_ne!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                2
            ),
            EvidenceEligibilityDispositionV1::EligibleCurrent
        );
    }

    #[test]
    fn contested_observations_remain_contested_across_lifecycle_changes() {
        let (mut ledger, generation, snapshot) = active_ledger();
        let t = transition(&generation, "suspend-1", ObserverStatusV1::Suspended, 2, None);
        ledger.record_transition(t);
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
                ObservationClassificationV1::ContradictoryIndependent,
                1,
                2
            ),
            EvidenceEligibilityDispositionV1::Contested
        );
    }

    #[test]
    fn divergent_lifecycle_transitions_are_rejected() {
        let (mut ledger, generation, _) = active_ledger();
        let first = transition(&generation, "suspend-1", ObserverStatusV1::Suspended, 2, None);
        let second = transition(&generation, "revoke-1", ObserverStatusV1::Revoked, 2, None);
        assert_eq!(
            ledger.record_transition(first),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_transition(second),
            LifecycleRecordDispositionV1::Conflict
        );
    }

    #[test]
    fn duplicate_lifecycle_transition_with_divergent_contents_is_rejected() {
        let (mut ledger, generation, _) = active_ledger();
        let first = transition(&generation, "suspend-1", ObserverStatusV1::Suspended, 2, None);
        let mut second = first.clone();
        second.reason = "different".into();
        assert_eq!(
            ledger.record_transition(first),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            ledger.record_transition(second),
            LifecycleRecordDispositionV1::Conflict
        );
    }

    #[test]
    fn out_of_order_transition_delivery_does_not_change_logical_chain() {
        let generation = generation("observer-A-g1", 1, None);
        let mut ordered = ObserverLifecycleLedgerV1::default();
        ordered.record_generation(generation.clone());
        let t1 = transition(&generation, "suspend-1", ObserverStatusV1::Suspended, 2, None);
        let mut t2 = transition(&generation, "revoke-1", ObserverStatusV1::Revoked, 3, None);
        t2.from_status = ObserverStatusV1::Suspended;
        assert_eq!(
            ordered.record_transition(t1.clone()),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            ordered.record_transition(t2.clone()),
            LifecycleRecordDispositionV1::Recorded
        );

        let mut reversed = ObserverLifecycleLedgerV1::default();
        reversed.record_generation(generation);
        assert_eq!(
            reversed.record_transition(t2.clone()),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            reversed.status_at_frontier("observer-A-g1", 3),
            None
        );
        assert_eq!(
            reversed.record_transition(t1),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            ordered.status_at_frontier("observer-A-g1", 3),
            reversed.status_at_frontier("observer-A-g1", 3)
        );
        assert_eq!(
            reversed.status_at_frontier("observer-A-g1", 3),
            Some(ObserverStatusV1::Revoked)
        );
    }

    #[test]
    fn impossible_status_regression_is_rejected_immediately() {
        let (mut ledger, generation, _) = active_ledger();
        let mut invalid = transition(
            &generation,
            "invalid-1",
            ObserverStatusV1::Suspended,
            2,
            None,
        );
        invalid.from_status = ObserverStatusV1::Revoked;
        assert_eq!(
            ledger.record_transition(invalid),
            LifecycleRecordDispositionV1::InsufficientEvidence
        );
    }

    #[test]
    fn out_of_order_dependency_snapshot_delivery_converges_after_predecessor_arrives() {
        let generation = generation("observer-A-g1", 1, None);
        let snapshot_one = snapshot(
            "snapshot-1",
            &generation.generation_id,
            1,
            None,
            &generation.evidence_root,
            &generation.custody_root,
            ObservationIndependenceV1::DeclaredIndependent,
        );
        let snapshot_two = snapshot(
            "snapshot-2",
            &generation.generation_id,
            2,
            Some("snapshot-1"),
            "new-evidence-root",
            &generation.custody_root,
            ObservationIndependenceV1::DeclaredIndependent,
        );

        let mut reversed = ObserverLifecycleLedgerV1::default();
        reversed.record_generation(generation.clone());
        assert_eq!(
            reversed.record_dependency_snapshot(snapshot_two.clone()),
            LifecycleRecordDispositionV1::Recorded
        );
        assert!(reversed.dependency_snapshot_at(&generation.generation_id, 2).is_none());
        assert_eq!(
            reversed.record_dependency_snapshot(snapshot_one.clone()),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            reversed
                .dependency_snapshot_at(&generation.generation_id, 2)
                .map(|snapshot| snapshot.snapshot_id.as_str()),
            Some("snapshot-2")
        );

        let mut ordered = ObserverLifecycleLedgerV1::default();
        ordered.record_generation(generation.clone());
        ordered.record_dependency_snapshot(snapshot_one);
        ordered.record_dependency_snapshot(snapshot_two);
        assert_eq!(
            reversed.dependency_snapshot_at(&generation.generation_id, 2),
            ordered.dependency_snapshot_at(&generation.generation_id, 2)
        );
    }

    #[test]
    fn authoritative_receipt_reconstruction_rejects_source_substitution() {
        let (ledger, generation, snapshot) = active_ledger();
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        let receipt = EvidenceEligibilityReceiptV1 {
            eligibility_id: "eligibility-1".into(),
            observation_id: "obs-1".into(),
            observer_id: generation.observer_id.clone(),
            observer_generation_id: generation.generation_id.clone(),
            observation_profile_id: generation.observation_profile_id.clone(),
            semantic_environment_root: generation.semantic_environment_root.clone(),
            dependency_snapshot_id: snapshot.snapshot_id.clone(),
            observation_frontier_root: e.observation.observed_frontier_root.clone(),
            observation_frontier_sequence: 1,
            current_frontier_root: "frontier-1".into(),
            current_frontier_sequence: 1,
            current_generation_id: Some(generation.generation_id.clone()),
            qualification_profile_id: profile().profile_id.clone(),
            provenance: ObserverEvidenceProvenanceV1::Live,
            classification: ObservationClassificationV1::CorroboratingIndependent,
            disposition: EvidenceEligibilityDispositionV1::EligibleCurrent,
            lifecycle_transition_ids: BTreeSet::new(),
            eligibility_commitment: String::new(),
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        };
        let mut receipt = receipt;
        receipt.eligibility_commitment = receipt.recomputed_commitment();

        assert!(verify_eligibility_receipt_provenance(
            &receipt, &e, &generation, &snapshot, &profile(), &ledger
        ));

        let mut forged_snapshot = receipt.clone();
        forged_snapshot.dependency_snapshot_id = "snapshot-substituted".into();
        assert!(!verify_eligibility_receipt_provenance(
            &forged_snapshot, &e, &generation, &snapshot, &profile(), &ledger
        ));

        let mut forged_generation = receipt.clone();
        forged_generation.observer_generation_id = "observer-A-g2".into();
        assert!(!verify_eligibility_receipt_provenance(
            &forged_generation, &e, &generation, &snapshot, &profile(), &ledger
        ));

        let mut changed_evidence = e.clone();
        changed_evidence.observer.custody_root = "custody-substituted".into();
        assert!(!verify_eligibility_receipt_provenance(
            &receipt, &changed_evidence, &generation, &snapshot, &profile(), &ledger
        ));
    }

    #[test]
    fn symthaea_proposal_without_qualified_transition_is_non_authoritative() {
        let proposal = ObserverLifecycleProposalV1 {
            proposal_id: "proposal-1".into(),
            observer_id: "observer-A".into(),
            predecessor_generation_id: Some("observer-A-g1".into()),
            proposed_generation_id: Some("observer-A-g2".into()),
            proposed_status: ObserverStatusV1::Active,
            analysis_commitment: "analysis-1".into(),
            qualified_transition_id: None,
            claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
        };
        assert!(proposal.structurally_valid());
        assert!(!lifecycle_proposal_is_authoritative(&proposal));
    }

    #[test]
    fn lifecycle_receipt_cannot_authorize_actuation() {
        let (ledger, generation, snapshot) = active_ledger();
        let e = evidence("obs-1", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        assert_eq!(
            eligibility(
                &ledger,
                &e,
                &generation,
                &snapshot,
                ObserverLifecycleUsePurposeV1::CurrentActuationAuthorization,
                ObservationClassificationV1::CorroboratingIndependent,
                1,
                1
            ),
            EvidenceEligibilityDispositionV1::BlockedAuthorization
        );
    }

    #[test]
    fn lifecycle_transition_cannot_mint_authority_capacity_or_consent() {
        let generation = generation("observer-A-g1", 1, None);
        let t = transition(&generation, "suspend-1", ObserverStatusV1::Suspended, 2, None);
        assert!(!lifecycle_transition_can_authorize_actuation(&t));
        assert!(!lifecycle_transition_can_mint_authority_capacity_or_consent(&t));
    }

    #[test]
    fn archived_lifecycle_evidence_remains_historical() {
        let (ledger, generation, snapshot) = active_ledger();
        let e = evidence("obs-archive", &generation, "frontier-1", ExternalObservedStateV1::Applied);
        assert_eq!(
            assess_evidence_eligibility(
                &e,
                &generation,
                &snapshot,
                &profile(),
                &ledger,
                ObservationClassificationV1::CorroboratingIndependent,
                ObserverEvidenceProvenanceV1::Archived,
                "frontier-1",
                1,
                "frontier-1",
                1,
                ObserverLifecycleUsePurposeV1::ArchiveHistoricalEvidence,
            ),
            EvidenceEligibilityDispositionV1::HistoricalOnly
        );
    }

    #[test]
    fn lifecycle_generation_registration_is_arrival_order_invariant() {
        let predecessor = generation("observer-A-g1", 1, None);
        let successor = generation("observer-A-g2", 2, Some("observer-A-g1"));

        let mut first = ObserverLifecycleLedgerV1::default();
        assert_eq!(
            first.record_generation(successor.clone()),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            first.record_generation(predecessor.clone()),
            LifecycleRecordDispositionV1::Recorded
        );

        let mut second = ObserverLifecycleLedgerV1::default();
        assert_eq!(
            second.record_generation(predecessor),
            LifecycleRecordDispositionV1::Recorded
        );
        assert_eq!(
            second.record_generation(successor),
            LifecycleRecordDispositionV1::Recorded
        );

        assert_eq!(
            first.generations.get("observer-A-g2"),
            second.generations.get("observer-A-g2")
        );
    }
}