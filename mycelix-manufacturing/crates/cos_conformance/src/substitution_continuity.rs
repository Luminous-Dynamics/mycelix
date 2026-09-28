//! Semantic recovery and cross-provider substitution continuity reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! A provider route is an implementation detail of one semantic effect. A route
//! change cannot create a new effect, resolve an unknown outcome by assumption,
//! or promote archived evidence into present authority.

use crate::archive_continuity::{
    assess_archive_rehydration, ArchiveRehydrationDispositionV1, HistoricalEvidenceProfileV1,
    SemanticArchiveManifestV1,
};
use crate::no_resurrection::SemanticTombstone;
use crate::stable_frontier::{
    reconstruction_matches_manifest, ColdStartManifestV1, ReconstructionReceiptV1,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const SEMANTIC_SUBSTITUTION_PROFILE_ID: &str = "INTEGRAL-SUBSTITUTION-REF-001";
pub const OUTCOME_CLAIM_CEILING: &str =
    "Provider observation only; no independent outcome-truth or authorization claim.";
pub const CONTINUITY_CLAIM_CEILING: &str =
    "Reference-model semantic continuity only; no exactly-once, current-authorization, or production-failover claim.";
pub const RECOVERY_INPUT_CLAIM_CEILING: &str =
    "Historical reconstruction input only; no current authority, provider execution, or production-safety claim.";

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IdempotencyScopeV1 {
    ProviderScoped,
    ContractWide,
}

/// Exact effect semantics against which every provider route is compared.
/// String-valued quantities are deliberately compared byte-for-byte; this
/// model does not normalize units, currencies, or decimal representations.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticEffectV1 {
    pub effect_id: String,
    pub lineage_id: String,
    pub generation_id: String,
    pub request_commitment: String,
    pub semantic_environment_root: String,
    pub effect_class: String,
    pub resource_id: String,
    pub tenant_id: String,
    pub amount: String,
    pub unit: String,
    pub authority_claim_id: String,
    pub consent_claim_id: String,
    pub idempotency_key: String,
}

impl SemanticEffectV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.effect_id)
            && non_empty(&self.lineage_id)
            && non_empty(&self.generation_id)
            && non_empty(&self.request_commitment)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.effect_class)
            && non_empty(&self.resource_id)
            && non_empty(&self.tenant_id)
            && non_empty(&self.amount)
            && non_empty(&self.unit)
            && non_empty(&self.authority_claim_id)
            && non_empty(&self.consent_claim_id)
            && non_empty(&self.idempotency_key)
    }
}

/// A closed semantic contract for a specific effect. Provider profiles are
/// explicitly allow-listed by provider identity and exact profile root.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticSubstitutionProfileV1 {
    pub profile_id: String,
    pub semantic_environment_root: String,
    pub request_commitment: String,
    pub effect_class: String,
    pub resource_id: String,
    pub tenant_id: String,
    pub amount: String,
    pub unit: String,
    pub authority_claim_id: String,
    pub consent_claim_id: String,
    pub idempotency_key: String,
    pub idempotency_scope: IdempotencyScopeV1,
    pub idempotency_policy_root: Option<String>,
    pub allow_retry_after_no_effect: bool,
    pub allow_same_provider_retry: bool,
    pub allowed_provider_profile_roots: BTreeMap<String, String>,
    pub profile_commitment: String,
}

impl SemanticSubstitutionProfileV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.profile_id)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.request_commitment)
            && non_empty(&self.effect_class)
            && non_empty(&self.resource_id)
            && non_empty(&self.tenant_id)
            && non_empty(&self.amount)
            && non_empty(&self.unit)
            && non_empty(&self.authority_claim_id)
            && non_empty(&self.consent_claim_id)
            && non_empty(&self.idempotency_key)
            && non_empty(&self.profile_commitment)
            && !self.allowed_provider_profile_roots.is_empty()
            && self
                .allowed_provider_profile_roots
                .iter()
                .all(|(provider, root)| non_empty(provider) && non_empty(root))
            && (self.idempotency_scope != IdempotencyScopeV1::ContractWide
                || self
                    .idempotency_policy_root
                    .as_deref()
                    .map(non_empty)
                    .unwrap_or(false))
    }
}

fn profile_matches_effect(
    effect: &SemanticEffectV1,
    profile: &SemanticSubstitutionProfileV1,
) -> bool {
    effect.request_commitment == profile.request_commitment
        && effect.semantic_environment_root == profile.semantic_environment_root
        && effect.effect_class == profile.effect_class
        && effect.resource_id == profile.resource_id
        && effect.tenant_id == profile.tenant_id
        && effect.amount == profile.amount
        && effect.unit == profile.unit
        && effect.authority_claim_id == profile.authority_claim_id
        && effect.consent_claim_id == profile.consent_claim_id
        && effect.idempotency_key == profile.idempotency_key
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderRouteV1 {
    pub route_id: String,
    pub effect_id: String,
    pub substitution_profile_id: String,
    pub provider_id: String,
    pub provider_profile_root: String,
    pub provider_operation_id: String,
    pub route_generation: u64,
    pub lifecycle_generation_id: String,
    pub request_commitment: String,
    pub semantic_environment_root: String,
    pub effect_class: String,
    pub resource_id: String,
    pub tenant_id: String,
    pub amount: String,
    pub unit: String,
    pub authority_claim_id: String,
    pub consent_claim_id: String,
    pub idempotency_key: String,
    pub route_frontier_root: String,
}

impl ProviderRouteV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.route_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.substitution_profile_id)
            && non_empty(&self.provider_id)
            && non_empty(&self.provider_profile_root)
            && non_empty(&self.provider_operation_id)
            && non_empty(&self.lifecycle_generation_id)
            && non_empty(&self.request_commitment)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.effect_class)
            && non_empty(&self.resource_id)
            && non_empty(&self.tenant_id)
            && non_empty(&self.amount)
            && non_empty(&self.unit)
            && non_empty(&self.authority_claim_id)
            && non_empty(&self.consent_claim_id)
            && non_empty(&self.idempotency_key)
            && non_empty(&self.route_frontier_root)
    }
}

fn route_matches_effect_and_profile(
    route: &ProviderRouteV1,
    effect: &SemanticEffectV1,
    profile: &SemanticSubstitutionProfileV1,
) -> bool {
    route.effect_id == effect.effect_id
        && route.substitution_profile_id == profile.profile_id
        && route.lifecycle_generation_id == effect.generation_id
        && route.request_commitment == effect.request_commitment
        && route.semantic_environment_root == effect.semantic_environment_root
        && route.effect_class == effect.effect_class
        && route.resource_id == effect.resource_id
        && route.tenant_id == effect.tenant_id
        && route.amount == effect.amount
        && route.unit == effect.unit
        && route.authority_claim_id == effect.authority_claim_id
        && route.consent_claim_id == effect.consent_claim_id
        && route.idempotency_key == effect.idempotency_key
        && route.provider_operation_id != effect.effect_id
        && profile
            .allowed_provider_profile_roots
            .get(&route.provider_id)
            .map(|root| root == &route.provider_profile_root)
            .unwrap_or(false)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProviderOutcomeKindV1 {
    /// Qualified evidence that the provider did not start the effect.
    NotStarted,
    /// A terminal rejection whose contract says no effect was applied.
    RejectedWithoutEffect,
    /// The provider reports an in-progress operation.
    Pending,
    /// The provider reports that the effect completed.
    Succeeded,
    /// The provider reports failure but cannot establish whether effects occurred.
    FailedWithPossibleEffect,
    /// No reliable terminal outcome is available (including an ambiguous timeout).
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderOutcomeV1 {
    pub outcome_id: String,
    pub effect_id: String,
    pub route_id: String,
    pub provider_id: String,
    pub provider_profile_root: String,
    pub request_commitment: String,
    pub idempotency_key: String,
    pub semantic_environment_root: String,
    pub observed_frontier_root: String,
    pub outcome_kind: ProviderOutcomeKindV1,
    pub evidence_root: String,
    pub claim_ceiling: String,
}

impl ProviderOutcomeV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.outcome_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.route_id)
            && non_empty(&self.provider_id)
            && non_empty(&self.provider_profile_root)
            && non_empty(&self.request_commitment)
            && non_empty(&self.idempotency_key)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.observed_frontier_root)
            && non_empty(&self.evidence_root)
            && self.claim_ceiling == OUTCOME_CLAIM_CEILING
    }
}

fn outcome_matches_route(outcome: &ProviderOutcomeV1, route: &ProviderRouteV1) -> bool {
    outcome.effect_id == route.effect_id
        && outcome.route_id == route.route_id
        && outcome.provider_id == route.provider_id
        && outcome.provider_profile_root == route.provider_profile_root
        && outcome.request_commitment == route.request_commitment
        && outcome.idempotency_key == route.idempotency_key
        && outcome.semantic_environment_root == route.semantic_environment_root
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutcomeResolutionKindV1 {
    NotApplied,
    Applied,
    Unresolved,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OutcomeResolutionV1 {
    pub resolution_id: String,
    pub effect_id: String,
    pub prior_outcome_id: String,
    pub prior_route_id: String,
    pub provider_id: String,
    pub provider_profile_root: String,
    pub request_commitment: String,
    pub idempotency_key: String,
    pub substitution_profile_id: String,
    pub semantic_environment_root: String,
    pub resolved_kind: OutcomeResolutionKindV1,
    pub evidence_root: String,
    pub resolution_commitment: String,
    pub claim_ceiling: String,
}

impl OutcomeResolutionV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.resolution_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.prior_outcome_id)
            && non_empty(&self.prior_route_id)
            && non_empty(&self.provider_id)
            && non_empty(&self.provider_profile_root)
            && non_empty(&self.request_commitment)
            && non_empty(&self.idempotency_key)
            && non_empty(&self.substitution_profile_id)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.evidence_root)
            && non_empty(&self.resolution_commitment)
            && self.claim_ceiling == CONTINUITY_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutcomeResolutionDispositionV1 {
    AcceptedNotApplied,
    AcceptedApplied,
    StillUnresolved,
    BlockedMismatch,
    InsufficientEvidence,
}

/// Validates only exact semantic bindings. The evidence root is opaque here;
/// this reference model does not verify signatures or provider truth.
pub fn assess_outcome_resolution(
    resolution: &OutcomeResolutionV1,
    effect: &SemanticEffectV1,
    profile: &SemanticSubstitutionProfileV1,
    route: &ProviderRouteV1,
    outcome: &ProviderOutcomeV1,
) -> OutcomeResolutionDispositionV1 {
    if !resolution.structurally_valid()
        || !effect.structurally_valid()
        || !profile.structurally_valid()
        || !route.structurally_valid()
        || !outcome.structurally_valid()
    {
        return OutcomeResolutionDispositionV1::InsufficientEvidence;
    }
    if !profile_matches_effect(effect, profile)
        || !route_matches_effect_and_profile(route, effect, profile)
        || !outcome_matches_route(outcome, route)
        || resolution.effect_id != effect.effect_id
        || resolution.prior_outcome_id != outcome.outcome_id
        || resolution.prior_route_id != route.route_id
        || resolution.provider_id != route.provider_id
        || resolution.provider_profile_root != route.provider_profile_root
        || resolution.request_commitment != effect.request_commitment
        || resolution.idempotency_key != effect.idempotency_key
        || resolution.substitution_profile_id != profile.profile_id
        || resolution.semantic_environment_root != effect.semantic_environment_root
    {
        return OutcomeResolutionDispositionV1::BlockedMismatch;
    }
    match (outcome.outcome_kind, resolution.resolved_kind) {
        (ProviderOutcomeKindV1::Succeeded, OutcomeResolutionKindV1::NotApplied)
        | (
            ProviderOutcomeKindV1::NotStarted | ProviderOutcomeKindV1::RejectedWithoutEffect,
            OutcomeResolutionKindV1::Applied,
        ) => OutcomeResolutionDispositionV1::BlockedMismatch,
        (_, OutcomeResolutionKindV1::NotApplied) => {
            OutcomeResolutionDispositionV1::AcceptedNotApplied
        }
        (_, OutcomeResolutionKindV1::Applied) => OutcomeResolutionDispositionV1::AcceptedApplied,
        (_, OutcomeResolutionKindV1::Unresolved) => {
            OutcomeResolutionDispositionV1::StillUnresolved
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CrossProviderIdempotencyWitnessV1 {
    pub witness_id: String,
    pub effect_id: String,
    pub request_commitment: String,
    pub idempotency_key: String,
    pub substitution_profile_id: String,
    pub semantic_environment_root: String,
    pub from_provider_id: String,
    pub from_provider_profile_root: String,
    pub to_provider_id: String,
    pub to_provider_profile_root: String,
    pub idempotency_policy_root: String,
    pub evidence_root: String,
    pub witness_commitment: String,
    pub claim_ceiling: String,
}

impl CrossProviderIdempotencyWitnessV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.witness_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.request_commitment)
            && non_empty(&self.idempotency_key)
            && non_empty(&self.substitution_profile_id)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.from_provider_id)
            && non_empty(&self.from_provider_profile_root)
            && non_empty(&self.to_provider_id)
            && non_empty(&self.to_provider_profile_root)
            && non_empty(&self.idempotency_policy_root)
            && non_empty(&self.evidence_root)
            && non_empty(&self.witness_commitment)
            && self.claim_ceiling == CONTINUITY_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IdempotencyWitnessDispositionV1 {
    AcceptedContractWide,
    BlockedProfile,
    BlockedBinding,
    InsufficientEvidence,
}

/// A contract-wide idempotency witness can permit continuation after an
/// uncertain outcome only when it binds both exact provider profiles and the
/// same effect/request/key. Provider-local idempotency is insufficient.
pub fn assess_cross_provider_idempotency_witness(
    witness: &CrossProviderIdempotencyWitnessV1,
    effect: &SemanticEffectV1,
    profile: &SemanticSubstitutionProfileV1,
    predecessor: &ProviderRouteV1,
    successor: &ProviderRouteV1,
) -> IdempotencyWitnessDispositionV1 {
    if !witness.structurally_valid()
        || !effect.structurally_valid()
        || !profile.structurally_valid()
        || !predecessor.structurally_valid()
        || !successor.structurally_valid()
    {
        return IdempotencyWitnessDispositionV1::InsufficientEvidence;
    }
    if profile.idempotency_scope != IdempotencyScopeV1::ContractWide
        || profile.idempotency_policy_root.as_deref()
            != Some(witness.idempotency_policy_root.as_str())
    {
        return IdempotencyWitnessDispositionV1::BlockedProfile;
    }
    if !profile_matches_effect(effect, profile)
        || !route_matches_effect_and_profile(predecessor, effect, profile)
        || !route_matches_effect_and_profile(successor, effect, profile)
        || predecessor.provider_id == successor.provider_id
        || witness.effect_id != effect.effect_id
        || witness.request_commitment != effect.request_commitment
        || witness.idempotency_key != effect.idempotency_key
        || witness.substitution_profile_id != profile.profile_id
        || witness.semantic_environment_root != effect.semantic_environment_root
        || witness.from_provider_id != predecessor.provider_id
        || witness.from_provider_profile_root != predecessor.provider_profile_root
        || witness.to_provider_id != successor.provider_id
        || witness.to_provider_profile_root != successor.provider_profile_root
    {
        return IdempotencyWitnessDispositionV1::BlockedBinding;
    }
    IdempotencyWitnessDispositionV1::AcceptedContractWide
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EffectContinuityReceiptV1 {
    pub continuity_id: String,
    pub effect_id: String,
    pub predecessor_route_id: String,
    pub predecessor_outcome_id: String,
    pub predecessor_provider_id: String,
    pub predecessor_provider_profile_root: String,
    pub predecessor_operation_id: String,
    pub successor_route_id: String,
    pub successor_provider_id: String,
    pub successor_provider_profile_root: String,
    pub successor_operation_id: String,
    pub predecessor_route_generation: u64,
    pub successor_route_generation: u64,
    pub lifecycle_generation_id: String,
    pub request_commitment: String,
    pub idempotency_key: String,
    pub substitution_profile_id: String,
    pub semantic_environment_root: String,
    pub predecessor_frontier_root: String,
    pub successor_frontier_root: String,
    pub transition_evidence_root: String,
    pub outcome_resolution_id: Option<String>,
    pub idempotency_witness_id: Option<String>,
    pub continuity_commitment: String,
    pub claim_ceiling: String,
}

impl EffectContinuityReceiptV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.continuity_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.predecessor_route_id)
            && non_empty(&self.predecessor_outcome_id)
            && non_empty(&self.predecessor_provider_id)
            && non_empty(&self.predecessor_provider_profile_root)
            && non_empty(&self.predecessor_operation_id)
            && non_empty(&self.successor_route_id)
            && non_empty(&self.successor_provider_id)
            && non_empty(&self.successor_provider_profile_root)
            && non_empty(&self.successor_operation_id)
            && non_empty(&self.lifecycle_generation_id)
            && non_empty(&self.request_commitment)
            && non_empty(&self.idempotency_key)
            && non_empty(&self.substitution_profile_id)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.predecessor_frontier_root)
            && non_empty(&self.successor_frontier_root)
            && non_empty(&self.transition_evidence_root)
            && non_empty(&self.continuity_commitment)
            && self.claim_ceiling == CONTINUITY_CLAIM_CEILING
            && self
                .outcome_resolution_id
                .as_deref()
                .map(non_empty)
                .unwrap_or(true)
            && self
                .idempotency_witness_id
                .as_deref()
                .map(non_empty)
                .unwrap_or(true)
    }
}

fn receipt_matches_routes(
    receipt: &EffectContinuityReceiptV1,
    effect: &SemanticEffectV1,
    profile: &SemanticSubstitutionProfileV1,
    predecessor: &ProviderRouteV1,
    outcome: &ProviderOutcomeV1,
    successor: &ProviderRouteV1,
    resolution: Option<&OutcomeResolutionV1>,
    witness: Option<&CrossProviderIdempotencyWitnessV1>,
) -> bool {
    receipt.effect_id == effect.effect_id
        && receipt.predecessor_route_id == predecessor.route_id
        && receipt.predecessor_outcome_id == outcome.outcome_id
        && receipt.predecessor_provider_id == predecessor.provider_id
        && receipt.predecessor_provider_profile_root == predecessor.provider_profile_root
        && receipt.predecessor_operation_id == predecessor.provider_operation_id
        && receipt.successor_route_id == successor.route_id
        && receipt.successor_provider_id == successor.provider_id
        && receipt.successor_provider_profile_root == successor.provider_profile_root
        && receipt.successor_operation_id == successor.provider_operation_id
        && receipt.predecessor_route_generation == predecessor.route_generation
        && receipt.successor_route_generation == successor.route_generation
        && receipt.lifecycle_generation_id == effect.generation_id
        && receipt.request_commitment == effect.request_commitment
        && receipt.idempotency_key == effect.idempotency_key
        && receipt.substitution_profile_id == profile.profile_id
        && receipt.semantic_environment_root == effect.semantic_environment_root
        && receipt.predecessor_frontier_root == outcome.observed_frontier_root
        && receipt.successor_frontier_root == successor.route_frontier_root
        && receipt.outcome_resolution_id.as_deref()
            == resolution.map(|item| item.resolution_id.as_str())
        && receipt.idempotency_witness_id.as_deref()
            == witness.map(|item| item.witness_id.as_str())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SubstitutionDispositionV1 {
    AcceptedAfterNoEffect,
    AcceptedAfterResolvedNotApplied,
    AcceptedByContractWideIdempotency,
    BlockedAlreadyApplied,
    BlockedUnknownOutcome,
    BlockedPendingOutcome,
    BlockedNoEffectRetryNotPermitted,
    BlockedProviderProfile,
    BlockedSemanticDrift,
    BlockedIdentity,
    BlockedLifecycle,
    BlockedRouteSequence,
    BlockedDuplicateOperation,
    BlockedResolutionMismatch,
    BlockedIdempotencyMismatch,
    BlockedReceiptMismatch,
    InsufficientEvidence,
}

/// Qualifies semantic continuity for one route transition. It does not
/// authorize the successor operation; current authority, consent, capacity,
/// and external-effect enforcement remain separate Mycelix decisions.
pub fn assess_effect_continuity(
    effect: &SemanticEffectV1,
    profile: &SemanticSubstitutionProfileV1,
    predecessor: &ProviderRouteV1,
    successor: &ProviderRouteV1,
    predecessor_outcome: &ProviderOutcomeV1,
    receipt: &EffectContinuityReceiptV1,
    resolution: Option<&OutcomeResolutionV1>,
    idempotency_witness: Option<&CrossProviderIdempotencyWitnessV1>,
    current_generation_id: &str,
    tombstone: Option<&SemanticTombstone>,
) -> SubstitutionDispositionV1 {
    if !effect.structurally_valid()
        || !profile.structurally_valid()
        || !predecessor.structurally_valid()
        || !successor.structurally_valid()
        || !predecessor_outcome.structurally_valid()
        || !receipt.structurally_valid()
        || !non_empty(current_generation_id)
    {
        return SubstitutionDispositionV1::InsufficientEvidence;
    }
    if !profile_matches_effect(effect, profile) {
        return SubstitutionDispositionV1::BlockedSemanticDrift;
    }
    if effect.generation_id != current_generation_id
        || tombstone
            .map(|item| {
                item.lineage_id == effect.lineage_id
                    && item.retired_generation_id == effect.generation_id
            })
            .unwrap_or(false)
    {
        return SubstitutionDispositionV1::BlockedLifecycle;
    }
    if predecessor.effect_id != effect.effect_id || successor.effect_id != effect.effect_id {
        return SubstitutionDispositionV1::BlockedIdentity;
    }
    if !profile
        .allowed_provider_profile_roots
        .get(&predecessor.provider_id)
        .map(|root| root == &predecessor.provider_profile_root)
        .unwrap_or(false)
        || !profile
            .allowed_provider_profile_roots
            .get(&successor.provider_id)
            .map(|root| root == &successor.provider_profile_root)
            .unwrap_or(false)
    {
        return SubstitutionDispositionV1::BlockedProviderProfile;
    }
    if !route_matches_effect_and_profile(predecessor, effect, profile)
        || !route_matches_effect_and_profile(successor, effect, profile)
    {
        return SubstitutionDispositionV1::BlockedSemanticDrift;
    }
    if predecessor_outcome.effect_id != effect.effect_id
        || !outcome_matches_route(predecessor_outcome, predecessor)
    {
        return SubstitutionDispositionV1::BlockedReceiptMismatch;
    }
    if !receipt_matches_routes(
        receipt,
        effect,
        profile,
        predecessor,
        predecessor_outcome,
        successor,
        resolution,
        idempotency_witness,
    ) {
        return SubstitutionDispositionV1::BlockedReceiptMismatch;
    }
    if predecessor.route_id == successor.route_id
        || predecessor.provider_operation_id == effect.effect_id
        || successor.provider_operation_id == effect.effect_id
        || (predecessor.provider_id == successor.provider_id
            && predecessor.provider_operation_id == successor.provider_operation_id)
    {
        return SubstitutionDispositionV1::BlockedDuplicateOperation;
    }
    let Some(expected_generation) = predecessor.route_generation.checked_add(1) else {
        return SubstitutionDispositionV1::BlockedRouteSequence;
    };
    if successor.route_generation != expected_generation {
        return SubstitutionDispositionV1::BlockedRouteSequence;
    }
    if predecessor.provider_id == successor.provider_id && !profile.allow_same_provider_retry {
        return SubstitutionDispositionV1::BlockedProviderProfile;
    }

    if resolution.is_some() && idempotency_witness.is_some() {
        return SubstitutionDispositionV1::BlockedResolutionMismatch;
    }
    if let Some(item) = resolution {
        match assess_outcome_resolution(item, effect, profile, predecessor, predecessor_outcome) {
            OutcomeResolutionDispositionV1::AcceptedNotApplied => {
                return SubstitutionDispositionV1::AcceptedAfterResolvedNotApplied;
            }
            OutcomeResolutionDispositionV1::AcceptedApplied => {
                return SubstitutionDispositionV1::BlockedAlreadyApplied;
            }
            OutcomeResolutionDispositionV1::StillUnresolved => {
                return SubstitutionDispositionV1::BlockedUnknownOutcome;
            }
            OutcomeResolutionDispositionV1::BlockedMismatch => {
                return SubstitutionDispositionV1::BlockedResolutionMismatch;
            }
            OutcomeResolutionDispositionV1::InsufficientEvidence => {
                return SubstitutionDispositionV1::InsufficientEvidence;
            }
        }
    }

    match predecessor_outcome.outcome_kind {
        ProviderOutcomeKindV1::Succeeded => SubstitutionDispositionV1::BlockedAlreadyApplied,
        ProviderOutcomeKindV1::NotStarted | ProviderOutcomeKindV1::RejectedWithoutEffect => {
            if profile.allow_retry_after_no_effect {
                SubstitutionDispositionV1::AcceptedAfterNoEffect
            } else {
                SubstitutionDispositionV1::BlockedNoEffectRetryNotPermitted
            }
        }
        ProviderOutcomeKindV1::Pending => {
            if let Some(witness) = idempotency_witness {
                match assess_cross_provider_idempotency_witness(
                    witness,
                    effect,
                    profile,
                    predecessor,
                    successor,
                ) {
                    IdempotencyWitnessDispositionV1::AcceptedContractWide => {
                        SubstitutionDispositionV1::AcceptedByContractWideIdempotency
                    }
                    IdempotencyWitnessDispositionV1::BlockedProfile
                    | IdempotencyWitnessDispositionV1::BlockedBinding => {
                        SubstitutionDispositionV1::BlockedIdempotencyMismatch
                    }
                    IdempotencyWitnessDispositionV1::InsufficientEvidence => {
                        SubstitutionDispositionV1::InsufficientEvidence
                    }
                }
            } else {
                SubstitutionDispositionV1::BlockedPendingOutcome
            }
        }
        ProviderOutcomeKindV1::FailedWithPossibleEffect | ProviderOutcomeKindV1::Unknown => {
            if let Some(witness) = idempotency_witness {
                match assess_cross_provider_idempotency_witness(
                    witness,
                    effect,
                    profile,
                    predecessor,
                    successor,
                ) {
                    IdempotencyWitnessDispositionV1::AcceptedContractWide => {
                        SubstitutionDispositionV1::AcceptedByContractWideIdempotency
                    }
                    IdempotencyWitnessDispositionV1::BlockedProfile
                    | IdempotencyWitnessDispositionV1::BlockedBinding => {
                        SubstitutionDispositionV1::BlockedIdempotencyMismatch
                    }
                    IdempotencyWitnessDispositionV1::InsufficientEvidence => {
                        SubstitutionDispositionV1::InsufficientEvidence
                    }
                }
            } else {
                SubstitutionDispositionV1::BlockedUnknownOutcome
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ProviderOperationKeyV1 {
    pub provider_id: String,
    pub provider_operation_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderOperationRecordV1 {
    pub key: ProviderOperationKeyV1,
    pub effect_id: String,
    pub route_id: String,
    pub provider_profile_root: String,
    pub request_commitment: String,
    pub lifecycle_generation_id: String,
}

impl ProviderOperationRecordV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.key.provider_id)
            && non_empty(&self.key.provider_operation_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.route_id)
            && non_empty(&self.provider_profile_root)
            && non_empty(&self.request_commitment)
            && non_empty(&self.lifecycle_generation_id)
            && self.key.provider_operation_id != self.effect_id
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProviderOperationDispositionV1 {
    Recorded,
    BlockedReplay,
    Conflict,
    InsufficientEvidence,
}

/// In-memory reference ledger only. A repeated provider operation key is
/// never interpreted as permission to execute or deliver the operation again.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderOperationLedgerV1 {
    pub records: BTreeMap<ProviderOperationKeyV1, ProviderOperationRecordV1>,
}

impl ProviderOperationLedgerV1 {
    pub fn record(
        &mut self,
        candidate: ProviderOperationRecordV1,
    ) -> ProviderOperationDispositionV1 {
        if !candidate.structurally_valid() {
            return ProviderOperationDispositionV1::InsufficientEvidence;
        }
        match self.records.get(&candidate.key) {
            Some(existing) if existing == &candidate => {
                ProviderOperationDispositionV1::BlockedReplay
            }
            Some(_) => ProviderOperationDispositionV1::Conflict,
            None => {
                self.records.insert(candidate.key.clone(), candidate);
                ProviderOperationDispositionV1::Recorded
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EffectConservationStateV1 {
    pub effect_ids: std::collections::BTreeSet<String>,
    pub authority_claim_ids: std::collections::BTreeSet<String>,
    pub capacity_claim_ids: std::collections::BTreeSet<String>,
    pub consent_claim_ids: std::collections::BTreeSet<String>,
}

/// A route transition must preserve the existing effect and conserved claims
/// exactly; creating or dropping one is a separate semantic transition.
pub fn substitution_preserves_claims(
    effect_id: &str,
    before: &EffectConservationStateV1,
    after: &EffectConservationStateV1,
) -> bool {
    non_empty(effect_id)
        && before.effect_ids.contains(effect_id)
        && after.effect_ids == before.effect_ids
        && after.authority_claim_ids == before.authority_claim_ids
        && after.capacity_claim_ids == before.capacity_claim_ids
        && after.consent_claim_ids == before.consent_claim_ids
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecoveryUsePurposeV1 {
    ReconstructionInput,
    CurrentProviderExecution,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecoveryInputDispositionV1 {
    UsableReconstructionInputOnly,
    BlockedCurrentAuthority,
    BlockedArchiveBinding,
    BlockedProfile,
    BlockedReconstruction,
    InsufficientEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArchiveRecoveryBindingV1 {
    pub archive_id: String,
    pub source_snapshot_root: String,
    pub source_frontier_root: String,
    pub semantic_environment_root: String,
    pub historical_profile_id: String,
    pub effect_id: String,
    pub request_commitment: String,
    pub lifecycle_generation_id: String,
    pub evidence_root: String,
    pub claim_ceiling: String,
}

impl ArchiveRecoveryBindingV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.archive_id)
            && non_empty(&self.source_snapshot_root)
            && non_empty(&self.source_frontier_root)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.historical_profile_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.request_commitment)
            && non_empty(&self.lifecycle_generation_id)
            && non_empty(&self.evidence_root)
            && self.claim_ceiling == RECOVERY_INPUT_CLAIM_CEILING
    }
}

/// Validates an archive solely as reconstruction input. Current provider
/// execution is blocked unconditionally here and must be authorized through
/// a separate, current Mycelix transition.
pub fn assess_recovered_effect_input(
    archive: &SemanticArchiveManifestV1,
    historical_profile: &HistoricalEvidenceProfileV1,
    manifest: &ColdStartManifestV1,
    reconstruction: &ReconstructionReceiptV1,
    binding: &ArchiveRecoveryBindingV1,
    effect: &SemanticEffectV1,
    available_history_roots: &std::collections::BTreeSet<String>,
    available_tombstone_ids: &std::collections::BTreeSet<String>,
    purpose: RecoveryUsePurposeV1,
) -> RecoveryInputDispositionV1 {
    if purpose == RecoveryUsePurposeV1::CurrentProviderExecution {
        return RecoveryInputDispositionV1::BlockedCurrentAuthority;
    }
    if !archive.structurally_valid()
        || !historical_profile.structurally_valid()
        || !effect.structurally_valid()
        || !binding.structurally_valid()
    {
        return RecoveryInputDispositionV1::InsufficientEvidence;
    }
    if binding.archive_id != archive.archive_id
        || binding.source_snapshot_root != archive.source_snapshot_root
        || binding.source_frontier_root != archive.source_frontier_root
        || binding.semantic_environment_root != archive.semantic_environment_root
        || binding.historical_profile_id != historical_profile.profile_id
        || binding.effect_id != effect.effect_id
        || binding.request_commitment != effect.request_commitment
        || binding.lifecycle_generation_id != effect.generation_id
    {
        return RecoveryInputDispositionV1::BlockedArchiveBinding;
    }
    if manifest.semantic_environment_root != effect.semantic_environment_root
        || historical_profile.semantic_environment_root != effect.semantic_environment_root
        || archive.semantic_environment_root != effect.semantic_environment_root
        || archive.profile_id != historical_profile.profile_id
        || manifest.reconstruction_profile_id != historical_profile.profile_id
    {
        return RecoveryInputDispositionV1::BlockedProfile;
    }
    if archive.source_snapshot_root != manifest.snapshot_root
        || archive.source_frontier_root != manifest.snapshot_frontier_root
        || !reconstruction_matches_manifest(reconstruction, manifest)
    {
        return RecoveryInputDispositionV1::BlockedArchiveBinding;
    }
    match assess_archive_rehydration(
        archive,
        historical_profile,
        manifest,
        available_history_roots,
        available_tombstone_ids,
    ) {
        ArchiveRehydrationDispositionV1::ReadyAsReconstructionInput => {
            RecoveryInputDispositionV1::UsableReconstructionInputOnly
        }
        ArchiveRehydrationDispositionV1::BlockedCurrentAuthority => {
            RecoveryInputDispositionV1::BlockedCurrentAuthority
        }
        ArchiveRehydrationDispositionV1::BlockedManifestMismatch => {
            RecoveryInputDispositionV1::BlockedProfile
        }
        ArchiveRehydrationDispositionV1::BlockedArchiveUse
        | ArchiveRehydrationDispositionV1::BlockedReconstruction => {
            RecoveryInputDispositionV1::BlockedReconstruction
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::archive_continuity::{
        ArchiveCompletenessV1, CurrentnessCeilingV1, HistoricalClaimClassV1,
    };
    use crate::stable_frontier::ReconstructionReceiptV1;
    use std::collections::{BTreeMap, BTreeSet};

    fn profile() -> SemanticSubstitutionProfileV1 {
        let mut allowed_provider_profile_roots = BTreeMap::new();
        allowed_provider_profile_roots.insert("provider-a".into(), "provider-profile-a".into());
        allowed_provider_profile_roots.insert("provider-b".into(), "provider-profile-b".into());
        SemanticSubstitutionProfileV1 {
            profile_id: "sub-profile-1".into(),
            semantic_environment_root: "environment-1".into(),
            request_commitment: "request-commitment-1".into(),
            effect_class: "transfer".into(),
            resource_id: "resource-1".into(),
            tenant_id: "tenant-1".into(),
            amount: "25.00".into(),
            unit: "MYC".into(),
            authority_claim_id: "authority-1".into(),
            consent_claim_id: "consent-1".into(),
            idempotency_key: "idem-1".into(),
            idempotency_scope: IdempotencyScopeV1::ProviderScoped,
            idempotency_policy_root: None,
            allow_retry_after_no_effect: true,
            allow_same_provider_retry: false,
            allowed_provider_profile_roots,
            profile_commitment: "sub-profile-commitment-1".into(),
        }
    }

    fn effect() -> SemanticEffectV1 {
        SemanticEffectV1 {
            effect_id: "effect-1".into(),
            lineage_id: "lineage-1".into(),
            generation_id: "generation-1".into(),
            request_commitment: "request-commitment-1".into(),
            semantic_environment_root: "environment-1".into(),
            effect_class: "transfer".into(),
            resource_id: "resource-1".into(),
            tenant_id: "tenant-1".into(),
            amount: "25.00".into(),
            unit: "MYC".into(),
            authority_claim_id: "authority-1".into(),
            consent_claim_id: "consent-1".into(),
            idempotency_key: "idem-1".into(),
        }
    }

    fn route(
        effect: &SemanticEffectV1,
        provider_id: &str,
        operation_id: &str,
        route_id: &str,
        route_generation: u64,
    ) -> ProviderRouteV1 {
        ProviderRouteV1 {
            route_id: route_id.into(),
            effect_id: effect.effect_id.clone(),
            substitution_profile_id: "sub-profile-1".into(),
            provider_id: provider_id.into(),
            provider_profile_root: format!("provider-profile-{}", &provider_id[9..]),
            provider_operation_id: operation_id.into(),
            route_generation,
            lifecycle_generation_id: effect.generation_id.clone(),
            request_commitment: effect.request_commitment.clone(),
            semantic_environment_root: effect.semantic_environment_root.clone(),
            effect_class: effect.effect_class.clone(),
            resource_id: effect.resource_id.clone(),
            tenant_id: effect.tenant_id.clone(),
            amount: effect.amount.clone(),
            unit: effect.unit.clone(),
            authority_claim_id: effect.authority_claim_id.clone(),
            consent_claim_id: effect.consent_claim_id.clone(),
            idempotency_key: effect.idempotency_key.clone(),
            route_frontier_root: format!("route-frontier-{route_generation}"),
        }
    }

    fn outcome(route: &ProviderRouteV1, kind: ProviderOutcomeKindV1) -> ProviderOutcomeV1 {
        ProviderOutcomeV1 {
            outcome_id: format!("outcome-{}", route.route_id),
            effect_id: route.effect_id.clone(),
            route_id: route.route_id.clone(),
            provider_id: route.provider_id.clone(),
            provider_profile_root: route.provider_profile_root.clone(),
            request_commitment: route.request_commitment.clone(),
            idempotency_key: route.idempotency_key.clone(),
            semantic_environment_root: route.semantic_environment_root.clone(),
            observed_frontier_root: format!("observed-{}", route.route_id),
            outcome_kind: kind,
            evidence_root: "provider-observation-root".into(),
            claim_ceiling: OUTCOME_CLAIM_CEILING.into(),
        }
    }

    fn receipt(
        effect: &SemanticEffectV1,
        predecessor: &ProviderRouteV1,
        prior: &ProviderOutcomeV1,
        successor: &ProviderRouteV1,
        resolution_id: Option<&str>,
        witness_id: Option<&str>,
    ) -> EffectContinuityReceiptV1 {
        EffectContinuityReceiptV1 {
            continuity_id: "continuity-1".into(),
            effect_id: effect.effect_id.clone(),
            predecessor_route_id: predecessor.route_id.clone(),
            predecessor_outcome_id: prior.outcome_id.clone(),
            predecessor_provider_id: predecessor.provider_id.clone(),
            predecessor_provider_profile_root: predecessor.provider_profile_root.clone(),
            predecessor_operation_id: predecessor.provider_operation_id.clone(),
            successor_route_id: successor.route_id.clone(),
            successor_provider_id: successor.provider_id.clone(),
            successor_provider_profile_root: successor.provider_profile_root.clone(),
            successor_operation_id: successor.provider_operation_id.clone(),
            predecessor_route_generation: predecessor.route_generation,
            successor_route_generation: successor.route_generation,
            lifecycle_generation_id: effect.generation_id.clone(),
            request_commitment: effect.request_commitment.clone(),
            idempotency_key: effect.idempotency_key.clone(),
            substitution_profile_id: "sub-profile-1".into(),
            semantic_environment_root: effect.semantic_environment_root.clone(),
            predecessor_frontier_root: prior.observed_frontier_root.clone(),
            successor_frontier_root: successor.route_frontier_root.clone(),
            transition_evidence_root: "transition-evidence-root".into(),
            outcome_resolution_id: resolution_id.map(str::to_owned),
            idempotency_witness_id: witness_id.map(str::to_owned),
            continuity_commitment: "continuity-commitment".into(),
            claim_ceiling: CONTINUITY_CLAIM_CEILING.into(),
        }
    }

    fn resolution(
        effect: &SemanticEffectV1,
        profile: &SemanticSubstitutionProfileV1,
        route: &ProviderRouteV1,
        prior: &ProviderOutcomeV1,
        kind: OutcomeResolutionKindV1,
    ) -> OutcomeResolutionV1 {
        OutcomeResolutionV1 {
            resolution_id: "resolution-1".into(),
            effect_id: effect.effect_id.clone(),
            prior_outcome_id: prior.outcome_id.clone(),
            prior_route_id: route.route_id.clone(),
            provider_id: route.provider_id.clone(),
            provider_profile_root: route.provider_profile_root.clone(),
            request_commitment: effect.request_commitment.clone(),
            idempotency_key: effect.idempotency_key.clone(),
            substitution_profile_id: profile.profile_id.clone(),
            semantic_environment_root: effect.semantic_environment_root.clone(),
            resolved_kind: kind,
            evidence_root: "resolution-evidence-root".into(),
            resolution_commitment: "resolution-commitment".into(),
            claim_ceiling: CONTINUITY_CLAIM_CEILING.into(),
        }
    }

    fn idempotency_witness(
        effect: &SemanticEffectV1,
        profile: &SemanticSubstitutionProfileV1,
        predecessor: &ProviderRouteV1,
        successor: &ProviderRouteV1,
    ) -> CrossProviderIdempotencyWitnessV1 {
        CrossProviderIdempotencyWitnessV1 {
            witness_id: "idempotency-witness-1".into(),
            effect_id: effect.effect_id.clone(),
            request_commitment: effect.request_commitment.clone(),
            idempotency_key: effect.idempotency_key.clone(),
            substitution_profile_id: profile.profile_id.clone(),
            semantic_environment_root: effect.semantic_environment_root.clone(),
            from_provider_id: predecessor.provider_id.clone(),
            from_provider_profile_root: predecessor.provider_profile_root.clone(),
            to_provider_id: successor.provider_id.clone(),
            to_provider_profile_root: successor.provider_profile_root.clone(),
            idempotency_policy_root: profile.idempotency_policy_root.clone().unwrap(),
            evidence_root: "idempotency-evidence-root".into(),
            witness_commitment: "idempotency-witness-commitment".into(),
            claim_ceiling: CONTINUITY_CLAIM_CEILING.into(),
        }
    }

    fn assess(
        effect: &SemanticEffectV1,
        profile: &SemanticSubstitutionProfileV1,
        predecessor: &ProviderRouteV1,
        successor: &ProviderRouteV1,
        prior: &ProviderOutcomeV1,
        receipt: &EffectContinuityReceiptV1,
        resolution: Option<&OutcomeResolutionV1>,
        witness: Option<&CrossProviderIdempotencyWitnessV1>,
        current_generation_id: &str,
        tombstone: Option<&SemanticTombstone>,
    ) -> SubstitutionDispositionV1 {
        assess_effect_continuity(
            effect,
            profile,
            predecessor,
            successor,
            prior,
            receipt,
            resolution,
            witness,
            current_generation_id,
            tombstone,
        )
    }

    fn archive_fixture() -> (
        SemanticArchiveManifestV1,
        HistoricalEvidenceProfileV1,
        ColdStartManifestV1,
        ReconstructionReceiptV1,
        ArchiveRecoveryBindingV1,
        BTreeSet<String>,
        BTreeSet<String>,
    ) {
        let mut allowed_claim_classes = BTreeSet::new();
        allowed_claim_classes.insert(HistoricalClaimClassV1::State);
        let historical_profile = HistoricalEvidenceProfileV1 {
            profile_id: "historical-profile-1".into(),
            semantic_environment_root: "environment-1".into(),
            allowed_claim_classes,
            currentness_ceiling: CurrentnessCeilingV1::ReconstructionInputOnly,
            reconstruction_allowed: true,
            profile_commitment: "historical-profile-commitment".into(),
        };
        let archive = SemanticArchiveManifestV1 {
            archive_id: "archive-1".into(),
            source_snapshot_root: "snapshot-1".into(),
            source_frontier_root: "frontier-1".into(),
            semantic_environment_root: "environment-1".into(),
            membership_epoch: 3,
            profile_id: historical_profile.profile_id.clone(),
            content_root: "archive-content-root".into(),
            inventory_root: "archive-inventory-root".into(),
            retained_tombstone_ids: BTreeSet::new(),
            completeness: ArchiveCompletenessV1::CompleteForProfile,
            manifest_commitment: "archive-manifest-commitment".into(),
        };
        let mut required_history_roots = BTreeSet::new();
        required_history_roots.insert("history-root-1".into());
        let manifest = ColdStartManifestV1 {
            node_id: "node-1".into(),
            incarnation_id: "incarnation-1".into(),
            semantic_environment_root: "environment-1".into(),
            membership_epoch: 3,
            snapshot_root: "snapshot-1".into(),
            snapshot_frontier_root: "frontier-1".into(),
            required_history_roots: required_history_roots.clone(),
            retained_tombstone_ids: BTreeSet::new(),
            reconstruction_profile_id: historical_profile.profile_id.clone(),
            normative_state_root: "state-root-1".into(),
            manifest_commitment: "cold-start-manifest-commitment".into(),
        };
        let reconstruction = ReconstructionReceiptV1 {
            node_id: manifest.node_id.clone(),
            incarnation_id: manifest.incarnation_id.clone(),
            semantic_environment_root: manifest.semantic_environment_root.clone(),
            snapshot_root: manifest.snapshot_root.clone(),
            source_frontier_root: manifest.snapshot_frontier_root.clone(),
            reconstructed_state_root: manifest.normative_state_root.clone(),
            retained_tombstone_ids: manifest.retained_tombstone_ids.clone(),
            claim_ceiling: "Reference reconstruction input only.".into(),
        };
        let effect = effect();
        let binding = ArchiveRecoveryBindingV1 {
            archive_id: archive.archive_id.clone(),
            source_snapshot_root: archive.source_snapshot_root.clone(),
            source_frontier_root: archive.source_frontier_root.clone(),
            semantic_environment_root: archive.semantic_environment_root.clone(),
            historical_profile_id: historical_profile.profile_id.clone(),
            effect_id: effect.effect_id.clone(),
            request_commitment: effect.request_commitment.clone(),
            lifecycle_generation_id: effect.generation_id.clone(),
            evidence_root: "archive-recovery-evidence-root".into(),
            claim_ceiling: RECOVERY_INPUT_CLAIM_CEILING.into(),
        };
        let available_history_roots = required_history_roots;
        let available_tombstone_ids = BTreeSet::new();
        (
            archive,
            historical_profile,
            manifest,
            reconstruction,
            binding,
            available_history_roots,
            available_tombstone_ids,
        )
    }

    #[test]
    fn unknown_provider_outcome_blocks_independent_failover() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::Unknown);
        let receipt = receipt(&effect, &predecessor, &prior, &successor, None, None);
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, None, None, "generation-1", None),
            SubstitutionDispositionV1::BlockedUnknownOutcome
        );
    }

    #[test]
    fn qualified_not_started_outcome_preserves_one_effect_identity() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::NotStarted);
        let receipt = receipt(&effect, &predecessor, &prior, &successor, None, None);
        assert_eq!(predecessor.effect_id, successor.effect_id);
        assert_ne!(predecessor.provider_operation_id, successor.provider_operation_id);
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, None, None, "generation-1", None),
            SubstitutionDispositionV1::AcceptedAfterNoEffect
        );
    }

    #[test]
    fn unknown_outcome_requires_exact_not_applied_resolution() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::Unknown);
        let resolution = resolution(&effect, &profile, &predecessor, &prior, OutcomeResolutionKindV1::NotApplied);
        let receipt = receipt(&effect, &predecessor, &prior, &successor, Some(&resolution.resolution_id), None);
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, Some(&resolution), None, "generation-1", None),
            SubstitutionDispositionV1::AcceptedAfterResolvedNotApplied
        );
    }

    #[test]
    fn applied_outcome_resolution_blocks_duplicate_effect() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::Unknown);
        let resolution = resolution(&effect, &profile, &predecessor, &prior, OutcomeResolutionKindV1::Applied);
        let receipt = receipt(&effect, &predecessor, &prior, &successor, Some(&resolution.resolution_id), None);
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, Some(&resolution), None, "generation-1", None),
            SubstitutionDispositionV1::BlockedAlreadyApplied
        );
    }

    #[test]
    fn contract_wide_idempotency_can_qualify_uncertain_continuation() {
        let effect = effect();
        let mut profile = profile();
        profile.idempotency_scope = IdempotencyScopeV1::ContractWide;
        profile.idempotency_policy_root = Some("cross-provider-idem-policy".into());
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::Unknown);
        let witness = idempotency_witness(&effect, &profile, &predecessor, &successor);
        let receipt = receipt(&effect, &predecessor, &prior, &successor, None, Some(&witness.witness_id));
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, None, Some(&witness), "generation-1", None),
            SubstitutionDispositionV1::AcceptedByContractWideIdempotency
        );
    }

    #[test]
    fn provider_scoped_idempotency_cannot_resolve_cross_provider_unknown() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::Unknown);
        let mut contract_profile = profile.clone();
        contract_profile.idempotency_scope = IdempotencyScopeV1::ContractWide;
        contract_profile.idempotency_policy_root = Some("cross-provider-idem-policy".into());
        let witness = idempotency_witness(&effect, &contract_profile, &predecessor, &successor);
        let receipt = receipt(&effect, &predecessor, &prior, &successor, None, Some(&witness.witness_id));
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, None, Some(&witness), "generation-1", None),
            SubstitutionDispositionV1::BlockedIdempotencyMismatch
        );
    }

    #[test]
    fn provider_profile_drift_blocks_even_when_visible_output_might_match() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let mut successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        successor.provider_profile_root = "unqualified-provider-profile".into();
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::NotStarted);
        let receipt = receipt(&effect, &predecessor, &prior, &successor, None, None);
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, None, None, "generation-1", None),
            SubstitutionDispositionV1::BlockedProviderProfile
        );
    }

    #[test]
    fn amount_unit_resource_and_tenant_drift_are_not_substitutable() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        for (field, changed) in [
            ("amount", "25"),
            ("unit", "USD"),
            ("resource", "resource-2"),
            ("tenant", "tenant-2"),
        ] {
            let mut successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
            match field {
                "amount" => successor.amount = changed.into(),
                "unit" => successor.unit = changed.into(),
                "resource" => successor.resource_id = changed.into(),
                "tenant" => successor.tenant_id = changed.into(),
                _ => unreachable!(),
            }
            let prior = outcome(&predecessor, ProviderOutcomeKindV1::NotStarted);
            let receipt = receipt(&effect, &predecessor, &prior, &successor, None, None);
            assert_eq!(
                assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, None, None, "generation-1", None),
                SubstitutionDispositionV1::BlockedSemanticDrift,
                "{field} drift was accepted"
            );
        }
    }

    #[test]
    fn resolution_for_another_effect_is_rejected() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::Unknown);
        let mut resolution = resolution(&effect, &profile, &predecessor, &prior, OutcomeResolutionKindV1::NotApplied);
        resolution.effect_id = "other-effect".into();
        let receipt = receipt(&effect, &predecessor, &prior, &successor, Some(&resolution.resolution_id), None);
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, Some(&resolution), None, "generation-1", None),
            SubstitutionDispositionV1::BlockedResolutionMismatch
        );
    }

    #[test]
    fn continuity_receipt_must_bind_exact_provider_operations() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::NotStarted);
        let mut receipt = receipt(&effect, &predecessor, &prior, &successor, None, None);
        receipt.successor_operation_id = "other-operation".into();
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, None, None, "generation-1", None),
            SubstitutionDispositionV1::BlockedReceiptMismatch
        );
    }

    #[test]
    fn route_generation_must_be_the_immediate_successor() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 2);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::NotStarted);
        let receipt = receipt(&effect, &predecessor, &prior, &successor, None, None);
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, None, None, "generation-1", None),
            SubstitutionDispositionV1::BlockedRouteSequence
        );
    }

    #[test]
    fn tombstoned_generation_cannot_continue_through_failover() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "operation-a", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::NotStarted);
        let receipt = receipt(&effect, &predecessor, &prior, &successor, None, None);
        let tombstone = SemanticTombstone {
            tombstone_id: "tombstone-1".into(),
            lineage_id: effect.lineage_id.clone(),
            retired_generation_id: effect.generation_id.clone(),
            retired_creation_event_id: "creation-event-1".into(),
            causal_frontier_root: "retired-frontier".into(),
            reason: crate::no_resurrection::TombstoneReason::Revoked,
            provenance_root: "tombstone-provenance".into(),
        };
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, None, None, "generation-1", Some(&tombstone)),
            SubstitutionDispositionV1::BlockedLifecycle
        );
    }

    #[test]
    fn provider_operation_identity_cannot_alias_semantic_effect_id() {
        let effect = effect();
        let profile = profile();
        let predecessor = route(&effect, "provider-a", "effect-1", "route-a", 0);
        let successor = route(&effect, "provider-b", "operation-b", "route-b", 1);
        let prior = outcome(&predecessor, ProviderOutcomeKindV1::NotStarted);
        let receipt = receipt(&effect, &predecessor, &prior, &successor, None, None);
        assert_eq!(
            assess(&effect, &profile, &predecessor, &successor, &prior, &receipt, None, None, "generation-1", None),
            SubstitutionDispositionV1::BlockedSemanticDrift
        );
    }

    #[test]
    fn provider_operation_ledger_blocks_exact_replay_and_conflicting_reuse() {
        let record = ProviderOperationRecordV1 {
            key: ProviderOperationKeyV1 {
                provider_id: "provider-a".into(),
                provider_operation_id: "operation-a".into(),
            },
            effect_id: "effect-1".into(),
            route_id: "route-a".into(),
            provider_profile_root: "provider-profile-a".into(),
            request_commitment: "request-commitment-1".into(),
            lifecycle_generation_id: "generation-1".into(),
        };
        let mut ledger = ProviderOperationLedgerV1::default();
        assert_eq!(ledger.record(record.clone()), ProviderOperationDispositionV1::Recorded);
        assert_eq!(ledger.record(record.clone()), ProviderOperationDispositionV1::BlockedReplay);
        let mut conflicting = record;
        conflicting.effect_id = "effect-2".into();
        assert_eq!(ledger.record(conflicting), ProviderOperationDispositionV1::Conflict);
    }

    #[test]
    fn route_transition_preserves_existing_conserved_claims_exactly() {
        let before = EffectConservationStateV1 {
            effect_ids: ["effect-1".into()].into_iter().collect(),
            authority_claim_ids: ["authority-1".into()].into_iter().collect(),
            capacity_claim_ids: ["capacity-1".into()].into_iter().collect(),
            consent_claim_ids: ["consent-1".into()].into_iter().collect(),
        };
        let after = before.clone();
        assert!(substitution_preserves_claims("effect-1", &before, &after));
        let mut duplicated = after.clone();
        duplicated.effect_ids.insert("effect-2".into());
        assert!(!substitution_preserves_claims("effect-1", &before, &duplicated));
        let mut reminted = after;
        reminted.capacity_claim_ids.insert("capacity-2".into());
        assert!(!substitution_preserves_claims("effect-1", &before, &reminted));
    }

    #[test]
    fn archive_input_cannot_authorize_current_provider_execution() {
        let (archive, historical_profile, manifest, reconstruction, binding, history, tombstones) =
            archive_fixture();
        let effect = effect();
        assert_eq!(
            assess_recovered_effect_input(
                &archive,
                &historical_profile,
                &manifest,
                &reconstruction,
                &binding,
                &effect,
                &history,
                &tombstones,
                RecoveryUsePurposeV1::CurrentProviderExecution,
            ),
            RecoveryInputDispositionV1::BlockedCurrentAuthority
        );
    }

    #[test]
    fn complete_archive_can_be_used_only_as_bound_reconstruction_input() {
        let (archive, historical_profile, manifest, reconstruction, binding, history, tombstones) =
            archive_fixture();
        let effect = effect();
        assert_eq!(
            assess_recovered_effect_input(
                &archive,
                &historical_profile,
                &manifest,
                &reconstruction,
                &binding,
                &effect,
                &history,
                &tombstones,
                RecoveryUsePurposeV1::ReconstructionInput,
            ),
            RecoveryInputDispositionV1::UsableReconstructionInputOnly
        );
    }

    #[test]
    fn stale_archive_binding_cannot_be_relabelled_as_recovery_of_another_frontier() {
        let (mut archive, historical_profile, manifest, reconstruction, mut binding, history, tombstones) =
            archive_fixture();
        let effect = effect();
        archive.source_frontier_root = "stale-frontier".into();
        binding.source_frontier_root = "stale-frontier".into();
        assert_eq!(
            assess_recovered_effect_input(
                &archive,
                &historical_profile,
                &manifest,
                &reconstruction,
                &binding,
                &effect,
                &history,
                &tombstones,
                RecoveryUsePurposeV1::ReconstructionInput,
            ),
            RecoveryInputDispositionV1::BlockedArchiveBinding
        );
    }
}
