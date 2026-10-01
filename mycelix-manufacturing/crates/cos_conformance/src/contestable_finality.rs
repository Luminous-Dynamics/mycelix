//! Contestable multi-observer external-finality reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! D6M separates provider-reported outcomes from independently qualified
//! external finality. D6N adds an explicit evidence-contest layer: observer
//! identity is not independence, copied evidence is not corroboration, and
//! conflict resolution is a new semantic transition rather than a winner
//! chosen by arrival order or observer count.

use crate::effect_finality::{
    ExternalEffectObservationV1, ExternalFinalityStateV1, ExternalObservationSourceV1,
};
use crate::no_resurrection::SemanticTombstone;
use crate::substitution_continuity::{ProviderRouteV1, SemanticEffectV1};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const CONTESTABLE_FINALITY_CLAIM_CEILING: &str =
    "Contestable external-finality reference evidence only; no physical truth, settlement, or actuation authorization claim.";

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ExternalObserverRoleV1 {
    Provider,
    IndependentObserver,
    SettlementAuthority,
    ArchiveMirror,
    DerivedObserver,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ObservationIndependenceV1 {
    DeclaredIndependent,
    DeclaredDependent,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObservationClassificationV1 {
    CorroboratingIndependent,
    CorroboratingDependent,
    ContradictoryIndependent,
    ContradictoryDependent,
    Stale,
    Superseded,
    Incomparable,
    InsufficientEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObservationSetDispositionV1 {
    QualifiedEvidence,
    Contested,
    InsufficientEvidence,
    BlockedBinding,
    BlockedProfile,
    BlockedCurrentness,
    BlockedLifecycle,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinalityResolutionDispositionV1 {
    AcceptedCurrent,
    AcceptedHistorical,
    Contested,
    InsufficientEvidence,
    BlockedBinding,
    BlockedIndependence,
    BlockedCurrentness,
    BlockedLifecycle,
    BlockedProfile,
    BlockedAuthorization,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinalityContestDispositionV1 {
    Open,
    Resolved,
    InsufficientEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContestLedgerDispositionV1 {
    Recorded,
    BlockedDuplicate,
    Conflict,
    InsufficientEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalObserverProfileV1 {
    pub observer_id: String,
    pub role: ExternalObserverRoleV1,
    pub observation_method: String,
    pub provider_relationship: String,
    pub evidence_root: String,
    pub custody_root: String,
    pub upstream_observer_ids: BTreeSet<String>,
    pub upstream_evidence_roots: BTreeSet<String>,
    pub semantic_environment_root: String,
    pub observation_profile_id: String,
    pub independence: ObservationIndependenceV1,
    pub independence_commitment: String,
    pub claim_ceiling: String,
}

impl ExternalObserverProfileV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.observer_id)
            && non_empty(&self.observation_method)
            && non_empty(&self.provider_relationship)
            && non_empty(&self.evidence_root)
            && non_empty(&self.custody_root)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.observation_profile_id)
            && non_empty(&self.independence_commitment)
            && self.claim_ceiling == CONTESTABLE_FINALITY_CLAIM_CEILING
            && !self.upstream_observer_ids.contains(&self.observer_id)
            && !self.upstream_evidence_roots.contains(&self.evidence_root)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinalityQualificationProfileV1 {
    pub profile_id: String,
    pub semantic_environment_root: String,
    pub allowed_observation_sources: BTreeSet<ExternalObservationSourceV1>,
    pub required_independent_observations: u32,
    pub current_frontier_required: bool,
    pub provider_reports_may_satisfy_independence: bool,
    pub allow_explicit_conflict_resolution: bool,
    pub profile_commitment: String,
    pub claim_ceiling: String,
}

impl FinalityQualificationProfileV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.profile_id)
            && non_empty(&self.semantic_environment_root)
            && !self.allowed_observation_sources.is_empty()
            && non_empty(&self.profile_commitment)
            && self.claim_ceiling == CONTESTABLE_FINALITY_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalObservedEvidenceV1 {
    pub observation: ExternalEffectObservationV1,
    pub observer_id: String,
    pub observer: ExternalObserverProfileV1,
}

impl ExternalObservedEvidenceV1 {
    pub fn structurally_valid(&self) -> bool {
        self.observation.structurally_valid()
            && non_empty(&self.observer_id)
            && self.observer.structurally_valid()
            && self.observer_id == self.observer.observer_id
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalObservationSetV1 {
    pub set_id: String,
    pub effect_id: String,
    pub effect_lineage_id: String,
    pub lifecycle_generation_id: String,
    pub route_id: String,
    pub provider_id: String,
    pub provider_operation_id: String,
    pub provider_profile_root: String,
    pub semantic_environment_root: String,
    pub observation_frontier_root: String,
    pub qualification_profile_id: String,
    pub observation_ids: BTreeSet<String>,
    pub target_state: ExternalFinalityStateV1,
    pub set_commitment: String,
    pub claim_ceiling: String,
}

impl ExternalObservationSetV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.set_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.effect_lineage_id)
            && non_empty(&self.lifecycle_generation_id)
            && non_empty(&self.route_id)
            && non_empty(&self.provider_id)
            && non_empty(&self.provider_operation_id)
            && non_empty(&self.provider_profile_root)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.observation_frontier_root)
            && non_empty(&self.qualification_profile_id)
            && !self.observation_ids.is_empty()
            && non_empty(&self.set_commitment)
            && self.claim_ceiling == CONTESTABLE_FINALITY_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObservationAssessmentV1 {
    pub observation_id: String,
    pub observer_id: String,
    pub independence: ObservationIndependenceV1,
    pub classification: ObservationClassificationV1,
    pub evidence_root: String,
    pub custody_root: String,
    pub assessment_commitment: String,
    pub claim_ceiling: String,
}

impl ObservationAssessmentV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.observation_id)
            && non_empty(&self.observer_id)
            && non_empty(&self.evidence_root)
            && non_empty(&self.custody_root)
            && non_empty(&self.assessment_commitment)
            && self.claim_ceiling == CONTESTABLE_FINALITY_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObservationSetAssessmentV1 {
    pub set_id: String,
    pub disposition: ObservationSetDispositionV1,
    pub independent_count: u32,
    pub contradictory_independent_count: u32,
    pub dependent_count: u32,
    pub assessments: Vec<ObservationAssessmentV1>,
    pub assessment_commitment: String,
    pub claim_ceiling: String,
}

impl ObservationSetAssessmentV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.set_id)
            && non_empty(&self.assessment_commitment)
            && self.claim_ceiling == CONTESTABLE_FINALITY_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinalityResolutionReceiptV1 {
    pub resolution_id: String,
    pub effect_id: String,
    pub effect_lineage_id: String,
    pub lifecycle_generation_id: String,
    pub route_id: String,
    pub provider_id: String,
    pub provider_operation_id: String,
    pub provider_profile_root: String,
    pub semantic_environment_root: String,
    pub observation_set_id: String,
    pub observation_set_commitment: String,
    pub qualification_profile_id: String,
    pub disposition: FinalityResolutionDispositionV1,
    pub resolved_state: Option<ExternalFinalityStateV1>,
    pub required_independent_observations: u32,
    pub observation_frontier_root: String,
    pub resolver_id: String,
    pub qualification_transition_id: String,
    pub resolution_commitment: String,
    pub claim_ceiling: String,
}

impl FinalityResolutionReceiptV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.resolution_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.effect_lineage_id)
            && non_empty(&self.lifecycle_generation_id)
            && non_empty(&self.route_id)
            && non_empty(&self.provider_id)
            && non_empty(&self.provider_operation_id)
            && non_empty(&self.provider_profile_root)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.observation_set_id)
            && non_empty(&self.observation_set_commitment)
            && non_empty(&self.qualification_profile_id)
            && non_empty(&self.observation_frontier_root)
            && non_empty(&self.resolver_id)
            && non_empty(&self.qualification_transition_id)
            && non_empty(&self.resolution_commitment)
            && self.claim_ceiling == CONTESTABLE_FINALITY_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinalityContestV1 {
    pub contest_id: String,
    pub effect_id: String,
    pub lifecycle_generation_id: String,
    pub observation_set_id: String,
    pub observation_ids: BTreeSet<String>,
    pub contradictory_observation_ids: BTreeSet<String>,
    pub disposition: FinalityContestDispositionV1,
    pub resolution_id: Option<String>,
    pub contest_commitment: String,
    pub claim_ceiling: String,
}

impl FinalityContestV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.contest_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.lifecycle_generation_id)
            && non_empty(&self.observation_set_id)
            && !self.observation_ids.is_empty()
            && non_empty(&self.contest_commitment)
            && self.claim_ceiling == CONTESTABLE_FINALITY_CLAIM_CEILING
            && match self.disposition {
                FinalityContestDispositionV1::Open
                | FinalityContestDispositionV1::InsufficientEvidence => self.resolution_id.is_none(),
                FinalityContestDispositionV1::Resolved => self.resolution_id.is_some(),
            }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObserverEvidenceLedgerV1 {
    pub observers: BTreeMap<String, ExternalObserverProfileV1>,
    pub observations: BTreeMap<String, ExternalObservedEvidenceV1>,
    pub observation_sets: BTreeMap<String, ExternalObservationSetV1>,
    pub resolutions: BTreeMap<String, FinalityResolutionReceiptV1>,
    pub terminal_resolution_by_effect: BTreeMap<String, String>,
    pub contests: BTreeMap<String, FinalityContestV1>,
}

impl ObserverEvidenceLedgerV1 {
    pub fn record_observer(&mut self, observer: ExternalObserverProfileV1) -> ContestLedgerDispositionV1 {
        if !observer.structurally_valid() {
            return ContestLedgerDispositionV1::InsufficientEvidence;
        }
        match self.observers.get(&observer.observer_id) {
            Some(existing) if existing == &observer => ContestLedgerDispositionV1::BlockedDuplicate,
            Some(_) => ContestLedgerDispositionV1::Conflict,
            None => {
                self.observers.insert(observer.observer_id.clone(), observer);
                ContestLedgerDispositionV1::Recorded
            }
        }
    }

    pub fn record_observation(&mut self, evidence: ExternalObservedEvidenceV1) -> ContestLedgerDispositionV1 {
        if !evidence.structurally_valid() {
            return ContestLedgerDispositionV1::InsufficientEvidence;
        }
        let id = evidence.observation.observation_id.clone();
        match self.observations.get(&id) {
            Some(existing) if existing == &evidence => ContestLedgerDispositionV1::BlockedDuplicate,
            Some(_) => ContestLedgerDispositionV1::Conflict,
            None => {
                self.observations.insert(id, evidence);
                ContestLedgerDispositionV1::Recorded
            }
        }
    }

    pub fn record_set(&mut self, set: ExternalObservationSetV1) -> ContestLedgerDispositionV1 {
        if !set.structurally_valid() {
            return ContestLedgerDispositionV1::InsufficientEvidence;
        }
        match self.observation_sets.get(&set.set_id) {
            Some(existing) if existing == &set => ContestLedgerDispositionV1::BlockedDuplicate,
            Some(_) => ContestLedgerDispositionV1::Conflict,
            None => {
                self.observation_sets.insert(set.set_id.clone(), set);
                ContestLedgerDispositionV1::Recorded
            }
        }
    }

    pub fn record_resolution(&mut self, resolution: FinalityResolutionReceiptV1) -> ContestLedgerDispositionV1 {
        if !resolution.structurally_valid() {
            return ContestLedgerDispositionV1::InsufficientEvidence;
        }
        if self.resolutions.contains_key(&resolution.resolution_id) {
            return ContestLedgerDispositionV1::BlockedDuplicate;
        }
        if matches!(
            resolution.disposition,
            FinalityResolutionDispositionV1::AcceptedCurrent
                | FinalityResolutionDispositionV1::AcceptedHistorical
        ) {
            if self.terminal_resolution_by_effect.contains_key(&resolution.effect_id) {
                return ContestLedgerDispositionV1::Conflict;
            }
            self.terminal_resolution_by_effect
                .insert(resolution.effect_id.clone(), resolution.resolution_id.clone());
        }
        self.resolutions
            .insert(resolution.resolution_id.clone(), resolution);
        ContestLedgerDispositionV1::Recorded
    }

    pub fn record_contest(&mut self, contest: FinalityContestV1) -> ContestLedgerDispositionV1 {
        if !contest.structurally_valid() {
            return ContestLedgerDispositionV1::InsufficientEvidence;
        }
        match self.contests.get(&contest.contest_id) {
            Some(existing) if existing == &contest => ContestLedgerDispositionV1::BlockedDuplicate,
            Some(_) => ContestLedgerDispositionV1::Conflict,
            None => {
                self.contests.insert(contest.contest_id.clone(), contest);
                ContestLedgerDispositionV1::Recorded
            }
        }
    }
}

fn effect_matches(
    effect: &SemanticEffectV1,
    set: &ExternalObservationSetV1,
) -> bool {
    effect.effect_id == set.effect_id
        && effect.lineage_id == set.effect_lineage_id
        && effect.generation_id == set.lifecycle_generation_id
        && effect.semantic_environment_root == set.semantic_environment_root
}

fn route_matches(
    route: &ProviderRouteV1,
    set: &ExternalObservationSetV1,
) -> bool {
    route.effect_id == set.effect_id
        && route.route_id == set.route_id
        && route.provider_id == set.provider_id
        && route.provider_operation_id == set.provider_operation_id
        && route.provider_profile_root == set.provider_profile_root
        && route.lifecycle_generation_id == set.lifecycle_generation_id
        && route.semantic_environment_root == set.semantic_environment_root
}

fn observation_matches(
    evidence: &ExternalObservedEvidenceV1,
    set: &ExternalObservationSetV1,
) -> bool {
    let observation = &evidence.observation;
    observation.effect_id == set.effect_id
        && observation.effect_lineage_id == set.effect_lineage_id
        && observation.lifecycle_generation_id == set.lifecycle_generation_id
        && observation.route_id == set.route_id
        && observation.provider_id == set.provider_id
        && observation.provider_operation_id == set.provider_operation_id
        && observation.provider_profile_root == set.provider_profile_root
        && observation.semantic_environment_root == set.semantic_environment_root
        && observation.observed_frontier_root == set.observation_frontier_root
        && observation.source != ExternalObservationSourceV1::QualifiedIndependentReconciliation
}

fn profiles_share_dependency(a: &ExternalObserverProfileV1, b: &ExternalObserverProfileV1) -> bool {
    a.evidence_root == b.evidence_root
        || a.custody_root == b.custody_root
        || a.upstream_observer_ids.contains(&b.observer_id)
        || b.upstream_observer_ids.contains(&a.observer_id)
        || !a.upstream_observer_ids.is_disjoint(&b.upstream_observer_ids)
        || !a.upstream_evidence_roots.is_disjoint(&b.upstream_evidence_roots)
        || a.upstream_evidence_roots.contains(&b.evidence_root)
        || b.upstream_evidence_roots.contains(&a.evidence_root)
}

fn independent_candidate(
    profile: &FinalityQualificationProfileV1,
    evidence: &ExternalObservedEvidenceV1,
) -> bool {
    let source_ok = profile
        .allowed_observation_sources
        .contains(&evidence.observation.source);
    let provider_ok = evidence.observation.source != ExternalObservationSourceV1::ProviderReported
        || profile.provider_reports_may_satisfy_independence;
    source_ok
        && provider_ok
        && evidence.observer.independence == ObservationIndependenceV1::DeclaredIndependent
        && matches!(
            evidence.observer.role,
            ExternalObserverRoleV1::IndependentObserver
                | ExternalObserverRoleV1::SettlementAuthority
                | ExternalObserverRoleV1::Provider
        )
}

/// Reconstruct the D6N observation-set assessment from authoritative inputs.
///
/// This deliberately compares the complete deterministic result rather than
/// trusting the assessment's self-reported counts, classifications, or
/// commitment. A caller therefore cannot substitute a forged-but-self-consistent
/// assessment without changing the authoritative derivation inputs.
pub fn verify_observation_set_assessment_provenance(
    assessment: &ObservationSetAssessmentV1,
    effect: &SemanticEffectV1,
    route: &ProviderRouteV1,
    profile: &FinalityQualificationProfileV1,
    set: &ExternalObservationSetV1,
    evidence: &[ExternalObservedEvidenceV1],
    current_frontier_root: &str,
    live_generation_id: &str,
) -> bool {
    let expected = assess_observation_set(
        effect,
        route,
        profile,
        set,
        evidence,
        current_frontier_root,
        live_generation_id,
    );
    assessment == &expected
}

/// Reconstruct the authoritative D6N observation-set binding from the effect,
/// route, qualification profile, live lifecycle generation, current frontier, and
/// supplied evidence. The set commitment is deliberately not treated as proof:
/// an attacker who changes semantic fields can recompute a superficial commitment,
/// so the consumer must reconstruct the semantic binding itself.
pub fn verify_observation_set_provenance(
    set: &ExternalObservationSetV1,
    effect: &SemanticEffectV1,
    route: &ProviderRouteV1,
    profile: &FinalityQualificationProfileV1,
    evidence: &[ExternalObservedEvidenceV1],
    current_frontier_root: &str,
    live_generation_id: &str,
) -> bool {
    if !set.structurally_valid()
        || !effect.structurally_valid()
        || !route.structurally_valid()
        || !profile.structurally_valid()
        || !effect_matches(effect, set)
        || !route_matches(route, set)
        || set.qualification_profile_id != profile.profile_id
        || set.semantic_environment_root != effect.semantic_environment_root
        || set.lifecycle_generation_id != live_generation_id
        || (profile.current_frontier_required
            && set.observation_frontier_root != current_frontier_root)
    {
        return false;
    }

    let mut observed_ids = BTreeSet::new();
    let mut has_target_support = false;
    for item in evidence {
        if !item.structurally_valid()
            || !set.observation_ids.contains(&item.observation.observation_id)
            || !observation_matches(item, set)
        {
            continue;
        }
        observed_ids.insert(item.observation.observation_id.clone());
        has_target_support |= matches!(
            (set.target_state, item.observation.observed_state),
            (ExternalFinalityStateV1::Applied, crate::effect_finality::ExternalObservedStateV1::Applied)
                | (ExternalFinalityStateV1::NotApplied, crate::effect_finality::ExternalObservedStateV1::NotApplied)
                | (ExternalFinalityStateV1::NotApplied, crate::effect_finality::ExternalObservedStateV1::Reversed)
        );
    }

    observed_ids == set.observation_ids && has_target_support
}

pub fn assess_observation_set(
    effect: &SemanticEffectV1,
    route: &ProviderRouteV1,
    profile: &FinalityQualificationProfileV1,
    set: &ExternalObservationSetV1,
    evidence: &[ExternalObservedEvidenceV1],
    current_frontier_root: &str,
    live_generation_id: &str,
) -> ObservationSetAssessmentV1 {
    if !effect.structurally_valid()
        || !route.structurally_valid()
        || !profile.structurally_valid()
        || !set.structurally_valid()
        || !effect_matches(effect, set)
        || !route_matches(route, set)
        || set.qualification_profile_id != profile.profile_id
        || profile.semantic_environment_root != effect.semantic_environment_root
        || (profile.current_frontier_required
            && set.observation_frontier_root != current_frontier_root)
    {
        let disposition = if profile.current_frontier_required
            && set.observation_frontier_root != current_frontier_root
        {
            ObservationSetDispositionV1::BlockedCurrentness
        } else if set.lifecycle_generation_id != live_generation_id {
            ObservationSetDispositionV1::BlockedLifecycle
        } else if set.qualification_profile_id != profile.profile_id {
            ObservationSetDispositionV1::BlockedProfile
        } else {
            ObservationSetDispositionV1::BlockedBinding
        };
        return ObservationSetAssessmentV1 {
            set_id: set.set_id.clone(),
            disposition,
            independent_count: 0,
            contradictory_independent_count: 0,
            dependent_count: 0,
            assessments: Vec::new(),
            assessment_commitment: "blocked".to_owned(),
            claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.to_owned(),
        };
    }

    let mut by_id = BTreeMap::new();
    for item in evidence {
        if item.structurally_valid()
            && set.observation_ids.contains(&item.observation.observation_id)
        {
            by_id.insert(item.observation.observation_id.clone(), item);
        }
    }

    if by_id.len() != set.observation_ids.len() {
        return ObservationSetAssessmentV1 {
            set_id: set.set_id.clone(),
            disposition: ObservationSetDispositionV1::InsufficientEvidence,
            independent_count: 0,
            contradictory_independent_count: 0,
            dependent_count: 0,
            assessments: Vec::new(),
            assessment_commitment: "missing-observation".to_owned(),
            claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.to_owned(),
        };
    }

    let mut assessments = Vec::new();
    for item in by_id.values() {
        let o = &item.observation;
        let mut classification = if !observation_matches(item, set) {
            ObservationClassificationV1::Incomparable
        } else if profile.current_frontier_required && o.observed_frontier_root != current_frontier_root {
            ObservationClassificationV1::Stale
        } else if !profile.allowed_observation_sources.contains(&o.source) {
            ObservationClassificationV1::InsufficientEvidence
        } else if matches!(o.observed_state, crate::effect_finality::ExternalObservedStateV1::Unknown
            | crate::effect_finality::ExternalObservedStateV1::Contested)
        {
            ObservationClassificationV1::InsufficientEvidence
        } else {
            ObservationClassificationV1::CorroboratingDependent
        };

        if matches!(
            classification,
            ObservationClassificationV1::CorroboratingDependent
        ) {
            let independent = independent_candidate(profile, item);
            if independent {
                classification = match o.observed_state {
                    crate::effect_finality::ExternalObservedStateV1::Applied
                    if set.target_state == ExternalFinalityStateV1::Applied =>
                        ObservationClassificationV1::CorroboratingIndependent,
                    crate::effect_finality::ExternalObservedStateV1::NotApplied
                    if set.target_state == ExternalFinalityStateV1::NotApplied =>
                        ObservationClassificationV1::CorroboratingIndependent,
                    crate::effect_finality::ExternalObservedStateV1::Reversed
                    if set.target_state == ExternalFinalityStateV1::NotApplied =>
                        ObservationClassificationV1::CorroboratingIndependent,
                    _ => ObservationClassificationV1::ContradictoryIndependent,
                };
            } else {
                classification = match o.observed_state {
                    crate::effect_finality::ExternalObservedStateV1::Applied
                    if set.target_state == ExternalFinalityStateV1::Applied =>
                        ObservationClassificationV1::CorroboratingDependent,
                    crate::effect_finality::ExternalObservedStateV1::NotApplied
                    if set.target_state == ExternalFinalityStateV1::NotApplied =>
                        ObservationClassificationV1::CorroboratingDependent,
                    crate::effect_finality::ExternalObservedStateV1::Reversed
                    if set.target_state == ExternalFinalityStateV1::NotApplied =>
                        ObservationClassificationV1::CorroboratingDependent,
                    _ => ObservationClassificationV1::ContradictoryDependent,
                };
            }
        }

        assessments.push(ObservationAssessmentV1 {
            observation_id: o.observation_id.clone(),
            observer_id: item.observer_id.clone(),
            independence: item.observer.independence,
            classification,
            evidence_root: item.observer.evidence_root.clone(),
            custody_root: item.observer.custody_root.clone(),
            assessment_commitment: format!(
                "{}:{}:{}",
                o.observation_id, item.observer.evidence_root, item.observer.custody_root
            ),
            claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.to_owned(),
        });
    }

    for i in 0..assessments.len() {
        for j in (i + 1)..assessments.len() {
            let left = by_id.get(&assessments[i].observation_id).expect("assessment source exists");
            let right = by_id.get(&assessments[j].observation_id).expect("assessment source exists");
            if profiles_share_dependency(&left.observer, &right.observer) {
                for assessment in [&mut assessments[i], &mut assessments[j]] {
                    if matches!(
                        assessment.classification,
                        ObservationClassificationV1::CorroboratingIndependent
                    ) {
                        assessment.classification =
                            ObservationClassificationV1::CorroboratingDependent;
                    } else if matches!(
                        assessment.classification,
                        ObservationClassificationV1::ContradictoryIndependent
                    ) {
                        assessment.classification =
                            ObservationClassificationV1::ContradictoryDependent;
                    }
                }
            }
        }
    }

    let independent_count = assessments
        .iter()
        .filter(|a| matches!(a.classification, ObservationClassificationV1::CorroboratingIndependent))
        .count() as u32;
    let contradictory_independent_count = assessments
        .iter()
        .filter(|a| matches!(a.classification, ObservationClassificationV1::ContradictoryIndependent))
        .count() as u32;
    let dependent_count = assessments
        .iter()
        .filter(|a| {
            matches!(
                a.classification,
                ObservationClassificationV1::CorroboratingDependent
                    | ObservationClassificationV1::ContradictoryDependent
            )
        })
        .count() as u32;

    let disposition = if contradictory_independent_count > 0 {
        ObservationSetDispositionV1::Contested
    } else if independent_count >= profile.required_independent_observations {
        ObservationSetDispositionV1::QualifiedEvidence
    } else {
        ObservationSetDispositionV1::InsufficientEvidence
    };

    ObservationSetAssessmentV1 {
        set_id: set.set_id.clone(),
        disposition,
        independent_count,
        contradictory_independent_count,
        dependent_count,
        assessments,
        assessment_commitment: format!("assessment:{}", set.set_commitment),
        claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.to_owned(),
    }
}

pub fn assess_finality_resolution(
    effect: &SemanticEffectV1,
    route: &ProviderRouteV1,
    profile: &FinalityQualificationProfileV1,
    set: &ExternalObservationSetV1,
    assessment: &ObservationSetAssessmentV1,
    receipt: &FinalityResolutionReceiptV1,
    current_frontier_root: &str,
    live_generation_id: &str,
    tombstone: Option<&SemanticTombstone>,
) -> FinalityResolutionDispositionV1 {
    if !receipt.structurally_valid()
        || !effect_matches(effect, set)
        || !route_matches(route, set)
        || receipt.effect_id != effect.effect_id
        || receipt.effect_lineage_id != effect.lineage_id
        || receipt.lifecycle_generation_id != effect.generation_id
        || receipt.route_id != route.route_id
        || receipt.provider_id != route.provider_id
        || receipt.provider_operation_id != route.provider_operation_id
        || receipt.provider_profile_root != route.provider_profile_root
        || receipt.semantic_environment_root != effect.semantic_environment_root
        || receipt.observation_set_id != set.set_id
        || receipt.observation_set_commitment != set.set_commitment
        || receipt.qualification_profile_id != profile.profile_id
        || receipt.required_independent_observations != profile.required_independent_observations
        || receipt.observation_frontier_root != set.observation_frontier_root
    {
        return FinalityResolutionDispositionV1::BlockedBinding;
    }

    if receipt.disposition == FinalityResolutionDispositionV1::BlockedAuthorization {
        return FinalityResolutionDispositionV1::BlockedAuthorization;
    }

    if effect.generation_id != live_generation_id
        || tombstone.is_some_and(|t| t.retired_generation_id == effect.generation_id)
    {
        return FinalityResolutionDispositionV1::BlockedLifecycle;
    }

    if profile.semantic_environment_root != effect.semantic_environment_root {
        return FinalityResolutionDispositionV1::BlockedProfile;
    }

    if profile.current_frontier_required && receipt.observation_frontier_root != current_frontier_root {
        return FinalityResolutionDispositionV1::BlockedCurrentness;
    }

    if receipt.resolver_id.is_empty()
        || set.observation_ids.contains(&receipt.resolver_id)
    {
        return FinalityResolutionDispositionV1::BlockedIndependence;
    }

    if assessment.contradictory_independent_count > 0 {
        if !profile.allow_explicit_conflict_resolution
            || receipt.qualification_transition_id.trim().is_empty()
        {
            return FinalityResolutionDispositionV1::Contested;
        }
    }

    if assessment.independent_count < profile.required_independent_observations {
        return FinalityResolutionDispositionV1::InsufficientEvidence;
    }

    if receipt.resolved_state.is_none() {
        return FinalityResolutionDispositionV1::InsufficientEvidence;
    }

    match receipt.disposition {
        FinalityResolutionDispositionV1::AcceptedCurrent
            if profile.current_frontier_required =>
        {
            FinalityResolutionDispositionV1::AcceptedCurrent
        }
        FinalityResolutionDispositionV1::AcceptedCurrent => {
            FinalityResolutionDispositionV1::AcceptedHistorical
        }
        FinalityResolutionDispositionV1::AcceptedHistorical => {
            FinalityResolutionDispositionV1::AcceptedHistorical
        }
        FinalityResolutionDispositionV1::Contested => FinalityResolutionDispositionV1::Contested,
        FinalityResolutionDispositionV1::InsufficientEvidence => {
            FinalityResolutionDispositionV1::InsufficientEvidence
        }
        FinalityResolutionDispositionV1::BlockedBinding => FinalityResolutionDispositionV1::BlockedBinding,
        FinalityResolutionDispositionV1::BlockedIndependence => FinalityResolutionDispositionV1::BlockedIndependence,
        FinalityResolutionDispositionV1::BlockedCurrentness => FinalityResolutionDispositionV1::BlockedCurrentness,
        FinalityResolutionDispositionV1::BlockedLifecycle => FinalityResolutionDispositionV1::BlockedLifecycle,
        FinalityResolutionDispositionV1::BlockedProfile => FinalityResolutionDispositionV1::BlockedProfile,
        FinalityResolutionDispositionV1::BlockedAuthorization => FinalityResolutionDispositionV1::BlockedAuthorization,
    }
}

pub fn archive_resolution_is_historical_only(
    effect_id: &str,
    resolution: &FinalityResolutionReceiptV1,
    archive_effect_id: &str,
) -> FinalityResolutionDispositionV1 {
    if !non_empty(effect_id)
        || !resolution.structurally_valid()
        || effect_id != archive_effect_id
        || resolution.effect_id != archive_effect_id
    {
        return FinalityResolutionDispositionV1::BlockedBinding;
    }
    FinalityResolutionDispositionV1::AcceptedHistorical
}

pub fn finality_resolution_cannot_authorize(
    resolution: &FinalityResolutionReceiptV1,
    requested_purpose: &str,
) -> bool {
    resolution.structurally_valid()
        && non_empty(requested_purpose)
        && requested_purpose == "CurrentActuationAuthorization"
}

pub fn compensation_may_reference_resolution(
    predecessor_effect_id: &str,
    resolution: &FinalityResolutionReceiptV1,
) -> bool {
    non_empty(predecessor_effect_id)
        && resolution.structurally_valid()
        && resolution.effect_id == predecessor_effect_id
        && matches!(
            resolution.disposition,
            FinalityResolutionDispositionV1::AcceptedCurrent
                | FinalityResolutionDispositionV1::AcceptedHistorical
        )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::effect_finality::{
        ExternalObservedStateV1, ExternalObservationSourceV1,
    };
    use std::collections::BTreeSet;

    fn effect() -> SemanticEffectV1 {
        SemanticEffectV1 {
            effect_id: "effect-1".into(),
            lineage_id: "lineage-1".into(),
            generation_id: "generation-1".into(),
            request_commitment: "request-1".into(),
            semantic_environment_root: "env-1".into(),
            effect_class: "payment".into(),
            resource_id: "resource-1".into(),
            tenant_id: "tenant-1".into(),
            amount: "10".into(),
            unit: "unit".into(),
            authority_claim_id: "authority-1".into(),
            consent_claim_id: "consent-1".into(),
            idempotency_key: "idem-1".into(),
        }
    }

    fn route() -> ProviderRouteV1 {
        ProviderRouteV1 {
            route_id: "route-1".into(),
            effect_id: "effect-1".into(),
            substitution_profile_id: "sub-profile-1".into(),
            provider_id: "provider-1".into(),
            provider_profile_root: "provider-profile-1".into(),
            provider_operation_id: "operation-1".into(),
            route_generation: 1,
            lifecycle_generation_id: "generation-1".into(),
            request_commitment: "request-1".into(),
            semantic_environment_root: "env-1".into(),
            effect_class: "payment".into(),
            resource_id: "resource-1".into(),
            tenant_id: "tenant-1".into(),
            amount: "10".into(),
            unit: "unit".into(),
            authority_claim_id: "authority-1".into(),
            consent_claim_id: "consent-1".into(),
            idempotency_key: "idem-1".into(),
            route_frontier_root: "frontier-1".into(),
        }
    }

    fn profile() -> FinalityQualificationProfileV1 {
        FinalityQualificationProfileV1 {
            profile_id: "qual-1".into(),
            semantic_environment_root: "env-1".into(),
            allowed_observation_sources: [
                ExternalObservationSourceV1::ProviderReported,
                ExternalObservationSourceV1::IndependentObserver,
                ExternalObservationSourceV1::SettlementAuthority,
            ]
            .into_iter()
            .collect(),
            required_independent_observations: 2,
            current_frontier_required: true,
            provider_reports_may_satisfy_independence: false,
            allow_explicit_conflict_resolution: true,
            profile_commitment: "profile-commitment".into(),
            claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
        }
    }

    fn observer(id: &str, evidence: &str, custody: &str) -> ExternalObserverProfileV1 {
        ExternalObserverProfileV1 {
            observer_id: id.into(),
            role: ExternalObserverRoleV1::IndependentObserver,
            observation_method: "independent-state-read".into(),
            provider_relationship: "external".into(),
            evidence_root: evidence.into(),
            custody_root: custody.into(),
            upstream_observer_ids: BTreeSet::new(),
            upstream_evidence_roots: BTreeSet::new(),
            semantic_environment_root: "env-1".into(),
            observation_profile_id: "obs-profile-1".into(),
            independence: ObservationIndependenceV1::DeclaredIndependent,
            independence_commitment: format!("indep:{id}"),
            claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
        }
    }

    fn evidence(
        id: &str,
        observer: ExternalObserverProfileV1,
        state: ExternalObservedStateV1,
    ) -> ExternalObservedEvidenceV1 {
        ExternalObservedEvidenceV1 {
            observation: ExternalEffectObservationV1 {
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
                observed_frontier_root: "frontier-1".into(),
                observed_state: state,
                source: ExternalObservationSourceV1::IndependentObserver,
                evidence_root: observer.evidence_root.clone(),
                claim_ceiling: crate::effect_finality::EXTERNAL_FINALITY_CLAIM_CEILING.into(),
            },
            observer,
        }
    }

    fn set(ids: &[&str]) -> ExternalObservationSetV1 {
        ExternalObservationSetV1 {
            set_id: "set-1".into(),
            effect_id: "effect-1".into(),
            effect_lineage_id: "lineage-1".into(),
            lifecycle_generation_id: "generation-1".into(),
            route_id: "route-1".into(),
            provider_id: "provider-1".into(),
            provider_operation_id: "operation-1".into(),
            provider_profile_root: "provider-profile-1".into(),
            semantic_environment_root: "env-1".into(),
            observation_frontier_root: "frontier-1".into(),
            qualification_profile_id: "qual-1".into(),
            observation_ids: ids.iter().map(|id| (*id).to_owned()).collect(),
            target_state: ExternalFinalityStateV1::Applied,
            set_commitment: "set-commitment".into(),
            claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
        }
    }

    fn receipt(disposition: FinalityResolutionDispositionV1) -> FinalityResolutionReceiptV1 {
        FinalityResolutionReceiptV1 {
            resolution_id: "resolution-1".into(),
            effect_id: "effect-1".into(),
            effect_lineage_id: "lineage-1".into(),
            lifecycle_generation_id: "generation-1".into(),
            route_id: "route-1".into(),
            provider_id: "provider-1".into(),
            provider_operation_id: "operation-1".into(),
            provider_profile_root: "provider-profile-1".into(),
            semantic_environment_root: "env-1".into(),
            observation_set_id: "set-1".into(),
            observation_set_commitment: "set-commitment".into(),
            qualification_profile_id: "qual-1".into(),
            disposition,
            resolved_state: Some(ExternalFinalityStateV1::Applied),
            required_independent_observations: 2,
            observation_frontier_root: "frontier-1".into(),
            resolver_id: "resolver-1".into(),
            qualification_transition_id: "transition-1".into(),
            resolution_commitment: "resolution-commitment".into(),
            claim_ceiling: CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
        }
    }

    #[test]
    fn observation_and_observer_identity_are_distinct() {
        let observer_profile = observer("observer-A", "evidence-1", "custody-1");
        let mut item = evidence(
            "observation-1",
            observer_profile,
            ExternalObservedStateV1::Applied,
        );
        item.observer_id = "observer-A".into();
        assert_ne!(item.observation.observation_id, item.observer_id);
        assert!(item.structurally_valid());
    }

    #[test]
    fn independent_observers_can_qualify_evidence() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &set(&["obs-1", "obs-2"]),
            &[e1, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.disposition, ObservationSetDispositionV1::QualifiedEvidence);
        assert_eq!(result.independent_count, 2);
    }

    #[test]
    fn authoritative_assessment_rejects_self_consistent_semantic_substitution() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let expected = assess_observation_set(
            &effect(), &route(), &profile(), &s, &[e1.clone(), e2.clone()],
            "frontier-1", "generation-1",
        );
        assert!(verify_observation_set_assessment_provenance(
            &expected, &effect(), &route(), &profile(), &s,
            &[e1.clone(), e2.clone()], "frontier-1", "generation-1",
        ));

        let mut forged = expected.clone();
        forged.assessments[0].classification =
            ObservationClassificationV1::ContradictoryIndependent;
        forged.assessment_commitment = expected.assessment_commitment.clone();
        assert!(!verify_observation_set_assessment_provenance(
            &forged, &effect(), &route(), &profile(), &s,
            &[e1, e2], "frontier-1", "generation-1",
        ));
    }

    #[test]
    fn provider_report_does_not_satisfy_independent_requirement() {
        let mut e = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        e.observation.source = ExternalObservationSourceV1::ProviderReported;
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &set(&["obs-1", "obs-2"]),
            &[e, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.independent_count, 1);
        assert_eq!(result.disposition, ObservationSetDispositionV1::InsufficientEvidence);
    }

    #[test]
    fn shared_evidence_root_is_not_independent() {
        let e1 = evidence("obs-1", observer("obs-1", "same-root", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "same-root", "custody-2"), ExternalObservedStateV1::Applied);
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &set(&["obs-1", "obs-2"]),
            &[e1, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.independent_count, 0);
        assert_eq!(result.dependent_count, 2);
        assert_eq!(result.disposition, ObservationSetDispositionV1::InsufficientEvidence);
    }

    #[test]
    fn shared_custody_root_is_not_independent() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "same-custody"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "same-custody"), ExternalObservedStateV1::Applied);
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &set(&["obs-1", "obs-2"]),
            &[e1, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.independent_count, 0);
    }

    #[test]
    fn independent_contradiction_remains_contested() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::NotApplied);
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &set(&["obs-1", "obs-2"]),
            &[e1, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.contradictory_independent_count, 1);
        assert_eq!(result.disposition, ObservationSetDispositionV1::Contested);
    }

    #[test]
    fn dependent_contradiction_cannot_be_promoted_by_count() {
        let e1 = evidence("obs-1", observer("obs-1", "same-root", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "same-root", "custody-2"), ExternalObservedStateV1::NotApplied);
        let e3 = evidence("obs-3", observer("obs-3", "same-root", "custody-3"), ExternalObservedStateV1::NotApplied);
        let result = assess_observation_set(
            &effect(), &route(), &profile(),
            &set(&["obs-1", "obs-2", "obs-3"]), &[e1, e2, e3], "frontier-1", "generation-1",
        );
        assert_eq!(result.independent_count, 0);
        assert_eq!(result.contradictory_independent_count, 0);
        assert_eq!(result.disposition, ObservationSetDispositionV1::InsufficientEvidence);
    }

    #[test]
    fn stale_current_frontier_is_blocked() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let mut s = set(&["obs-1", "obs-2"]);
        s.observation_frontier_root = "old-frontier".into();
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &s, &[e1, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.disposition, ObservationSetDispositionV1::BlockedCurrentness);
    }

    #[test]
    fn wrong_effect_binding_is_blocked() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let mut s = set(&["obs-1", "obs-2"]);
        s.effect_id = "other-effect".into();
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &s, &[e1, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.disposition, ObservationSetDispositionV1::BlockedBinding);
    }

    #[test]
    fn wrong_provider_operation_is_not_comparable() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let mut s = set(&["obs-1", "obs-2"]);
        s.provider_operation_id = "other-operation".into();
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &s, &[e1, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.disposition, ObservationSetDispositionV1::BlockedBinding);
    }

    #[test]
    fn tombstoned_generation_cannot_gain_current_resolution() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let a = assess_observation_set(
            &effect(), &route(), &profile(), &s, &[e1, e2], "frontier-1", "generation-1",
        );
        let tombstone = SemanticTombstone {
            tombstone_id: "tombstone-1".into(),
            lineage_id: "lineage-1".into(),
            retired_generation_id: "generation-1".into(),
            retired_creation_event_id: "event-1".into(),
            causal_frontier_root: "frontier-1".into(),
            reason: crate::no_resurrection::TombstoneReason::Revoked,
            provenance_root: "provenance-1".into(),
        };
        let disposition = assess_finality_resolution(
            &effect(), &route(), &profile(), &s, &a, &receipt(FinalityResolutionDispositionV1::AcceptedCurrent),
            "frontier-1", "generation-1", Some(&tombstone),
        );
        assert_eq!(disposition, FinalityResolutionDispositionV1::BlockedLifecycle);
    }

    #[test]
    fn explicit_resolution_can_resolve_conflict_without_erasing_history() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::NotApplied);
        let s = set(&["obs-1", "obs-2"]);
        let a = assess_observation_set(
            &effect(), &route(), &profile(), &s, &[e1, e2], "frontier-1", "generation-1",
        );
        assert_eq!(a.disposition, ObservationSetDispositionV1::Contested);
        let disposition = assess_finality_resolution(
            &effect(), &route(), &profile(), &s, &a,
            &receipt(FinalityResolutionDispositionV1::AcceptedCurrent),
            "frontier-1", "generation-1", None,
        );
        assert_eq!(disposition, FinalityResolutionDispositionV1::AcceptedCurrent);
        assert!(a.assessments.iter().any(|x| matches!(
            x.classification,
            ObservationClassificationV1::ContradictoryIndependent
        )));
    }

    #[test]
    fn resolver_cannot_be_an_observer() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let a = assess_observation_set(
            &effect(), &route(), &profile(), &s, &[e1, e2], "frontier-1", "generation-1",
        );
        let mut r = receipt(FinalityResolutionDispositionV1::AcceptedCurrent);
        r.resolver_id = "obs-1".into();
        assert_eq!(
            assess_finality_resolution(&effect(), &route(), &profile(), &s, &a, &r, "frontier-1", "generation-1", None),
            FinalityResolutionDispositionV1::BlockedIndependence
        );
    }

    #[test]
    fn arrival_order_does_not_change_assessment() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::NotApplied);
        let s = set(&["obs-1", "obs-2"]);
        let a1 = assess_observation_set(&effect(), &route(), &profile(), &s, &[e1.clone(), e2.clone()], "frontier-1", "generation-1");
        let a2 = assess_observation_set(&effect(), &route(), &profile(), &s, &[e2, e1], "frontier-1", "generation-1");
        assert_eq!(a1.disposition, a2.disposition);
        assert_eq!(a1.independent_count, a2.independent_count);
        assert_eq!(a1.contradictory_independent_count, a2.contradictory_independent_count);
    }

    #[test]
    fn finality_resolution_cannot_authorize_actuation() {
        let r = receipt(FinalityResolutionDispositionV1::AcceptedCurrent);
        assert!(finality_resolution_cannot_authorize(&r, "CurrentActuationAuthorization"));
    }

    #[test]
    fn archive_resolution_is_historical_only() {
        let r = receipt(FinalityResolutionDispositionV1::AcceptedCurrent);
        assert_eq!(
            archive_resolution_is_historical_only("effect-1", &r, "effect-1"),
            FinalityResolutionDispositionV1::AcceptedHistorical
        );
    }

    #[test]
    fn compensation_requires_qualified_resolution() {
        let r = receipt(FinalityResolutionDispositionV1::Contested);
        assert!(!compensation_may_reference_resolution("effect-1", &r));
        let r = receipt(FinalityResolutionDispositionV1::AcceptedHistorical);
        assert!(compensation_may_reference_resolution("effect-1", &r));
    }

    #[test]
    fn ledger_rejects_conflicting_terminal_resolutions() {
        let mut ledger = ObserverEvidenceLedgerV1::default();
        let first = receipt(FinalityResolutionDispositionV1::AcceptedCurrent);
        let second = FinalityResolutionReceiptV1 {
            resolution_id: "resolution-2".into(),
            ..first.clone()
        };
        assert_eq!(ledger.record_resolution(first), ContestLedgerDispositionV1::Recorded);
        assert_eq!(ledger.record_resolution(second), ContestLedgerDispositionV1::Conflict);
    }

    #[test]
    fn ledger_rejects_divergent_duplicate_observer() {
        let mut ledger = ObserverEvidenceLedgerV1::default();
        let first = observer("obs-1", "evidence-1", "custody-1");
        let mut second = first.clone();
        second.evidence_root = "different".into();
        assert_eq!(ledger.record_observer(first), ContestLedgerDispositionV1::Recorded);
        assert_eq!(ledger.record_observer(second), ContestLedgerDispositionV1::Conflict);
    }

    #[test]
    fn symthaea_like_proposal_without_qualification_is_non_authoritative() {
        let r = receipt(FinalityResolutionDispositionV1::InsufficientEvidence);
        assert_eq!(r.disposition, FinalityResolutionDispositionV1::InsufficientEvidence);
        assert!(!compensation_may_reference_resolution("effect-1", &r));
    }

    #[test]
    fn equal_visible_state_with_distinct_evidence_roots_remains_distinct() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        assert_ne!(e1.observation.evidence_root, e2.observation.evidence_root);
        assert_ne!(e1.observer.evidence_root, e2.observer.evidence_root);
    }

    #[test]
    fn archive_mirror_does_not_become_independent_by_identity() {
        let mut e = evidence("obs-1", observer("obs-1", "archive-root", "archive-custody"), ExternalObservedStateV1::Applied);
        e.observer.role = ExternalObserverRoleV1::ArchiveMirror;
        e.observer.independence = ObservationIndependenceV1::DeclaredIndependent;
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &set(&["obs-1", "obs-2"]), &[e, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.independent_count, 1);
    }

    #[test]
    fn unknown_independence_is_conservative() {
        let mut e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        e1.observer.independence = ObservationIndependenceV1::Unknown;
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &set(&["obs-1", "obs-2"]), &[e1, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.independent_count, 1);
        assert_eq!(result.disposition, ObservationSetDispositionV1::InsufficientEvidence);
    }

    #[test]
    fn upstream_dependency_blocks_independence() {
        let mut o2 = observer("obs-2", "evidence-2", "custody-2");
        o2.upstream_observer_ids.insert("obs-1".into());
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", o2, ExternalObservedStateV1::Applied);
        let result = assess_observation_set(
            &effect(), &route(), &profile(), &set(&["obs-1", "obs-2"]), &[e1, e2], "frontier-1", "generation-1",
        );
        assert_eq!(result.independent_count, 0);
    }

    #[test]
    fn malformed_resolution_cannot_pass() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let s = set(&["obs-1", "obs-2"]);
        let a = assess_observation_set(&effect(), &route(), &profile(), &s, &[e1, e2], "frontier-1", "generation-1");
        let mut r = receipt(FinalityResolutionDispositionV1::AcceptedCurrent);
        r.resolution_commitment.clear();
        assert_eq!(
            assess_finality_resolution(&effect(), &route(), &profile(), &s, &a, &r, "frontier-1", "generation-1", None),
            FinalityResolutionDispositionV1::BlockedBinding
        );
    }

    #[test]
    fn current_resolution_requires_current_frontier() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::Applied);
        let mut s = set(&["obs-1", "obs-2"]);
        s.observation_frontier_root = "frontier-old".into();
        let a = assess_observation_set(&effect(), &route(), &profile(), &s, &[e1, e2], "frontier-new", "generation-1");
        assert_eq!(a.disposition, ObservationSetDispositionV1::BlockedCurrentness);
    }

    #[test]
    fn conflict_receipt_requires_explicit_transition() {
        let e1 = evidence("obs-1", observer("obs-1", "evidence-1", "custody-1"), ExternalObservedStateV1::Applied);
        let e2 = evidence("obs-2", observer("obs-2", "evidence-2", "custody-2"), ExternalObservedStateV1::NotApplied);
        let s = set(&["obs-1", "obs-2"]);
        let a = assess_observation_set(&effect(), &route(), &profile(), &s, &[e1, e2], "frontier-1", "generation-1");
        let mut p = profile();
        p.allow_explicit_conflict_resolution = false;
        let mut r = receipt(FinalityResolutionDispositionV1::AcceptedCurrent);
        assert_eq!(
            assess_finality_resolution(&effect(), &route(), &p, &s, &a, &r, "frontier-1", "generation-1", None),
            FinalityResolutionDispositionV1::Contested
        );
        r.qualification_transition_id = "transition-2".into();
        assert_eq!(
            assess_finality_resolution(&effect(), &route(), &profile(), &s, &a, &r, "frontier-1", "generation-1", None),
            FinalityResolutionDispositionV1::AcceptedCurrent
        );
    }
}
