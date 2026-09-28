//! External-effect finality and compensation-lineage reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! A provider-reported outcome is evidence about a provider operation. It is
//! not, by itself, independently qualified truth about the external effect.
//! Finality is a separate, effect-specific evidence transition. Reversal,
//! refund, remediation, and correction are new semantic effects linked to the
//! original effect; they never rewrite its identity or history.

use crate::archive_continuity::ArchiveRecoveryBindingV1;
use crate::no_resurrection::SemanticTombstone;
use crate::substitution_continuity::{
    EffectConservationStateV1, ProviderOutcomeKindV1, ProviderOutcomeV1, ProviderRouteV1,
    SemanticEffectV1, SemanticSubstitutionProfileV1,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const EXTERNAL_FINALITY_CLAIM_CEILING: &str =
    "External-effect finality evidence only; no provider truth, legal settlement, or actuation authorization claim.";
pub const COMPENSATION_CLAIM_CEILING: &str =
    "Compensation lineage reference semantics only; no physical delivery, accounting finality, or production safety claim.";
pub const LEDGER_CLAIM_CEILING: &str =
    "In-memory external-effect evidence ledger only; no durable storage or distributed-consensus claim.";
pub const ARCHIVE_FINALITY_CLAIM_CEILING: &str =
    "Historical archive boundary only; no current external finality or actuation authorization claim.";

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ExternalObservationSourceV1 {
    /// A provider assertion. This can corroborate an outcome but cannot alone
    /// establish independent external finality.
    ProviderReported,
    IndependentObserver,
    SettlementAuthority,
    QualifiedIndependentReconciliation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExternalObservedStateV1 {
    Unknown,
    NotApplied,
    Applied,
    Reversed,
    Contested,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExternalFinalityStateV1 {
    Applied,
    NotApplied,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinalityUsePurposeV1 {
    HistoricalEvidence,
    CurrentFinality,
    CurrentActuationAuthorization,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinalityDispositionV1 {
    AcceptedHistorical,
    AcceptedCurrent,
    BlockedProviderReported,
    BlockedUnresolved,
    BlockedOutcomeConflict,
    BlockedStaleObservation,
    BlockedProfile,
    BlockedLifecycle,
    BlockedReceiptMismatch,
    BlockedCurrentAuthorization,
    InsufficientEvidence,
}

/// An external observation binds one external-state assertion to the exact
/// Mycelix effect, provider route, provider operation, and observation frontier.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalEffectObservationV1 {
    pub observation_id: String,
    pub effect_id: String,
    pub effect_lineage_id: String,
    pub lifecycle_generation_id: String,
    pub route_id: String,
    pub provider_id: String,
    pub provider_operation_id: String,
    pub provider_profile_root: String,
    pub provider_outcome_id: String,
    pub request_commitment: String,
    pub idempotency_key: String,
    pub semantic_environment_root: String,
    pub observed_frontier_root: String,
    pub observed_state: ExternalObservedStateV1,
    pub source: ExternalObservationSourceV1,
    pub evidence_root: String,
    pub claim_ceiling: String,
}

impl ExternalEffectObservationV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.observation_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.effect_lineage_id)
            && non_empty(&self.lifecycle_generation_id)
            && non_empty(&self.route_id)
            && non_empty(&self.provider_id)
            && non_empty(&self.provider_operation_id)
            && non_empty(&self.provider_profile_root)
            && non_empty(&self.provider_outcome_id)
            && non_empty(&self.request_commitment)
            && non_empty(&self.idempotency_key)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.observed_frontier_root)
            && non_empty(&self.evidence_root)
            && self.claim_ceiling == EXTERNAL_FINALITY_CLAIM_CEILING
    }
}

fn effect_matches_profile(
    effect: &SemanticEffectV1,
    profile: &ExternalFinalityProfileV1,
) -> bool {
    effect.effect_id == profile.effect_id
        && effect.request_commitment == profile.request_commitment
        && effect.semantic_environment_root == profile.semantic_environment_root
        && effect.idempotency_key == profile.idempotency_key
}

fn route_matches_effect(
    route: &ProviderRouteV1,
    effect: &SemanticEffectV1,
    substitution_profile: &SemanticSubstitutionProfileV1,
) -> bool {
    route.effect_id == effect.effect_id
        && route.substitution_profile_id == substitution_profile.profile_id
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
        && substitution_profile
            .allowed_provider_profile_roots
            .get(&route.provider_id)
            .map(|root| root == &route.provider_profile_root)
            .unwrap_or(false)
}

fn outcome_matches_route(outcome: &ProviderOutcomeV1, route: &ProviderRouteV1) -> bool {
    outcome.effect_id == route.effect_id
        && outcome.route_id == route.route_id
        && outcome.provider_operation_id == route.provider_operation_id
        && outcome.provider_id == route.provider_id
        && outcome.provider_profile_root == route.provider_profile_root
        && outcome.request_commitment == route.request_commitment
        && outcome.idempotency_key == route.idempotency_key
        && outcome.semantic_environment_root == route.semantic_environment_root
}

fn observation_matches(
    observation: &ExternalEffectObservationV1,
    effect: &SemanticEffectV1,
    route: &ProviderRouteV1,
    outcome: &ProviderOutcomeV1,
) -> bool {
    observation.effect_id == effect.effect_id
        && observation.effect_lineage_id == effect.lineage_id
        && observation.lifecycle_generation_id == effect.generation_id
        && observation.route_id == route.route_id
        && observation.provider_id == route.provider_id
        && observation.provider_operation_id == route.provider_operation_id
        && observation.provider_profile_root == route.provider_profile_root
        && observation.provider_outcome_id == outcome.outcome_id
        && observation.request_commitment == effect.request_commitment
        && observation.idempotency_key == effect.idempotency_key
        && observation.semantic_environment_root == effect.semantic_environment_root
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalFinalityProfileV1 {
    pub profile_id: String,
    pub effect_id: String,
    pub substitution_profile_id: String,
    pub semantic_environment_root: String,
    pub request_commitment: String,
    pub idempotency_key: String,
    pub allowed_observation_sources: BTreeSet<ExternalObservationSourceV1>,
    pub required_finality_state: ExternalFinalityStateV1,
    pub current_frontier_required: bool,
    pub profile_commitment: String,
    pub claim_ceiling: String,
}

impl ExternalFinalityProfileV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.profile_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.substitution_profile_id)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.request_commitment)
            && non_empty(&self.idempotency_key)
            && !self.allowed_observation_sources.is_empty()
            && non_empty(&self.profile_commitment)
            && self.claim_ceiling == EXTERNAL_FINALITY_CLAIM_CEILING
    }
}

/// A finality receipt is an evidence claim, not an execution permit.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalFinalityReceiptV1 {
    pub receipt_id: String,
    pub effect_id: String,
    pub effect_lineage_id: String,
    pub lifecycle_generation_id: String,
    pub route_id: String,
    pub provider_id: String,
    pub provider_operation_id: String,
    pub provider_profile_root: String,
    pub provider_outcome_id: String,
    pub observation_id: String,
    pub observation_frontier_root: String,
    pub finality_profile_id: String,
    pub semantic_environment_root: String,
    pub finality_state: ExternalFinalityStateV1,
    pub evidence_root: String,
    pub finality_commitment: String,
    pub claim_ceiling: String,
}

impl ExternalFinalityReceiptV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.receipt_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.effect_lineage_id)
            && non_empty(&self.lifecycle_generation_id)
            && non_empty(&self.route_id)
            && non_empty(&self.provider_id)
            && non_empty(&self.provider_operation_id)
            && non_empty(&self.provider_profile_root)
            && non_empty(&self.provider_outcome_id)
            && non_empty(&self.observation_id)
            && non_empty(&self.observation_frontier_root)
            && non_empty(&self.finality_profile_id)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.evidence_root)
            && non_empty(&self.finality_commitment)
            && self.claim_ceiling == EXTERNAL_FINALITY_CLAIM_CEILING
    }
}

fn finality_receipt_matches(
    receipt: &ExternalFinalityReceiptV1,
    effect: &SemanticEffectV1,
    profile: &ExternalFinalityProfileV1,
    route: &ProviderRouteV1,
    outcome: &ProviderOutcomeV1,
    observation: &ExternalEffectObservationV1,
) -> bool {
    receipt.effect_id == effect.effect_id
        && receipt.effect_lineage_id == effect.lineage_id
        && receipt.lifecycle_generation_id == effect.generation_id
        && receipt.route_id == route.route_id
        && receipt.provider_id == route.provider_id
        && receipt.provider_operation_id == route.provider_operation_id
        && receipt.provider_profile_root == route.provider_profile_root
        && receipt.provider_outcome_id == outcome.outcome_id
        && receipt.observation_id == observation.observation_id
        && receipt.observation_frontier_root == observation.observed_frontier_root
        && receipt.finality_profile_id == profile.profile_id
        && receipt.semantic_environment_root == effect.semantic_environment_root
        && receipt.finality_state == profile.required_finality_state
}

fn outcome_is_compatible_with_finality(
    outcome: ProviderOutcomeKindV1,
    finality: ExternalFinalityStateV1,
) -> bool {
    match (outcome, finality) {
        (
            ProviderOutcomeKindV1::NotStarted | ProviderOutcomeKindV1::RejectedWithoutEffect,
            ExternalFinalityStateV1::Applied,
        ) => false,
        (ProviderOutcomeKindV1::Succeeded, ExternalFinalityStateV1::NotApplied) => false,
        _ => true,
    }
}

/// Qualifies external finality evidence without asserting that the evidence is
/// truthful. Provider-only assertions are deliberately insufficient.
pub fn assess_external_finality(
    effect: &SemanticEffectV1,
    substitution_profile: &SemanticSubstitutionProfileV1,
    finality_profile: &ExternalFinalityProfileV1,
    route: &ProviderRouteV1,
    outcome: &ProviderOutcomeV1,
    observation: &ExternalEffectObservationV1,
    receipt: &ExternalFinalityReceiptV1,
    current_frontier_root: &str,
    purpose: FinalityUsePurposeV1,
    tombstone: Option<&SemanticTombstone>,
) -> FinalityDispositionV1 {
    if !effect.structurally_valid()
        || !substitution_profile.structurally_valid()
        || !finality_profile.structurally_valid()
        || !route.structurally_valid()
        || !outcome.structurally_valid()
        || !observation.structurally_valid()
        || !receipt.structurally_valid()
        || !non_empty(current_frontier_root)
    {
        return FinalityDispositionV1::InsufficientEvidence;
    }

    if effect.generation_id.is_empty()
        || tombstone
            .map(|item| {
                item.lineage_id == effect.lineage_id
                    && item.retired_generation_id == effect.generation_id
            })
            .unwrap_or(false)
    {
        return FinalityDispositionV1::BlockedLifecycle;
    }

    if !effect_matches_profile(effect, finality_profile)
        || finality_profile.substitution_profile_id != substitution_profile.profile_id
        || finality_profile.semantic_environment_root != effect.semantic_environment_root
        || finality_profile.request_commitment != effect.request_commitment
        || finality_profile.idempotency_key != effect.idempotency_key
    {
        return FinalityDispositionV1::BlockedProfile;
    }

    if !route_matches_effect(route, effect, substitution_profile)
        || !outcome_matches_route(outcome, route)
        || !observation_matches(observation, effect, route, outcome)
        || !finality_receipt_matches(
            receipt,
            effect,
            finality_profile,
            route,
            outcome,
            observation,
        )
    {
        return FinalityDispositionV1::BlockedReceiptMismatch;
    }

    if observation.source == ExternalObservationSourceV1::ProviderReported {
        return FinalityDispositionV1::BlockedProviderReported;
    }
    if !finality_profile
        .allowed_observation_sources
        .contains(&observation.source)
    {
        return FinalityDispositionV1::BlockedProfile;
    }
    if matches!(
        observation.observed_state,
        ExternalObservedStateV1::Unknown | ExternalObservedStateV1::Contested
    ) {
        return FinalityDispositionV1::BlockedUnresolved;
    }
    if !outcome_is_compatible_with_finality(outcome.outcome_kind, receipt.finality_state) {
        return FinalityDispositionV1::BlockedOutcomeConflict;
    }

    match (receipt.finality_state, observation.observed_state) {
        (ExternalFinalityStateV1::Applied, ExternalObservedStateV1::Applied)
        | (ExternalFinalityStateV1::NotApplied, ExternalObservedStateV1::NotApplied) => {}
        _ => return FinalityDispositionV1::BlockedOutcomeConflict,
    }

    match purpose {
        FinalityUsePurposeV1::HistoricalEvidence => FinalityDispositionV1::AcceptedHistorical,
        FinalityUsePurposeV1::CurrentFinality => {
            if !finality_profile.current_frontier_required {
                return FinalityDispositionV1::BlockedProfile;
            }
            if observation.observed_frontier_root != current_frontier_root {
                return FinalityDispositionV1::BlockedStaleObservation;
            }
            FinalityDispositionV1::AcceptedCurrent
        }
        FinalityUsePurposeV1::CurrentActuationAuthorization => {
            FinalityDispositionV1::BlockedCurrentAuthorization
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompensationReasonV1 {
    Reversal,
    Refund,
    Remediation,
    Correction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompensationDispositionV1 {
    Accepted,
    BlockedIdentity,
    BlockedCause,
    BlockedLifecycle,
    BlockedMutation,
    BlockedConservation,
    BlockedDuplicate,
    BlockedFinalityMismatch,
    InsufficientEvidence,
}

/// Links a new compensation/reversal effect to the exact external evidence
/// that caused it. The predecessor remains immutable.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CompensationEffectLinkV1 {
    pub link_id: String,
    pub predecessor_effect_id: String,
    pub predecessor_lineage_id: String,
    pub predecessor_generation_id: String,
    pub predecessor_route_id: String,
    pub predecessor_provider_operation_id: String,
    pub compensation_effect_id: String,
    pub compensation_lineage_id: String,
    pub compensation_generation_id: String,
    pub cause_observation_id: String,
    pub cause_finality_receipt_id: Option<String>,
    pub cause_provider_id: String,
    pub cause_provider_profile_root: String,
    pub reason: CompensationReasonV1,
    pub semantic_environment_root: String,
    pub covered_capacity_claim_ids: BTreeSet<String>,
    pub accounting_root: String,
    pub link_commitment: String,
    pub claim_ceiling: String,
}

impl CompensationEffectLinkV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.link_id)
            && non_empty(&self.predecessor_effect_id)
            && non_empty(&self.predecessor_lineage_id)
            && non_empty(&self.predecessor_generation_id)
            && non_empty(&self.predecessor_route_id)
            && non_empty(&self.predecessor_provider_operation_id)
            && non_empty(&self.compensation_effect_id)
            && non_empty(&self.compensation_lineage_id)
            && non_empty(&self.compensation_generation_id)
            && non_empty(&self.cause_observation_id)
            && non_empty(&self.cause_provider_id)
            && non_empty(&self.cause_provider_profile_root)
            && non_empty(&self.semantic_environment_root)
            && non_empty(&self.accounting_root)
            && non_empty(&self.link_commitment)
            && self.claim_ceiling == COMPENSATION_CLAIM_CEILING
            && self
                .cause_finality_receipt_id
                .as_deref()
                .map(non_empty)
                .unwrap_or(true)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CompensationAssessmentV1 {
    pub disposition: CompensationDispositionV1,
    pub predecessor_effect_id: String,
    pub compensation_effect_id: String,
    pub claim_ceiling: String,
}

impl CompensationAssessmentV1 {
    fn new(
        disposition: CompensationDispositionV1,
        predecessor_effect_id: &str,
        compensation_effect_id: &str,
    ) -> Self {
        Self {
            disposition,
            predecessor_effect_id: predecessor_effect_id.to_owned(),
            compensation_effect_id: compensation_effect_id.to_owned(),
            claim_ceiling: COMPENSATION_CLAIM_CEILING.to_owned(),
        }
    }
}

fn conservation_allows_compensation(
    predecessor_effect_id: &str,
    compensation_effect_id: &str,
    covered_capacity_claim_ids: &BTreeSet<String>,
    before: &EffectConservationStateV1,
    after: &EffectConservationStateV1,
) -> bool {
    if !before.effect_ids.contains(predecessor_effect_id)
        || before.effect_ids.contains(compensation_effect_id)
        || !after.effect_ids.contains(predecessor_effect_id)
        || !after.effect_ids.contains(compensation_effect_id)
    {
        return false;
    }

    let mut expected_effects = before.effect_ids.clone();
    expected_effects.insert(compensation_effect_id.to_owned());

    before.effect_ids.iter().any(|id| id == predecessor_effect_id)
        && after.effect_ids == expected_effects
        && after.capacity_claim_ids == before.capacity_claim_ids
        && before
            .authority_claim_ids
            .is_subset(&after.authority_claim_ids)
        && before.consent_claim_ids.is_subset(&after.consent_claim_ids)
        && covered_capacity_claim_ids.is_subset(&before.capacity_claim_ids)
        && if before.capacity_claim_ids.is_empty() {
            covered_capacity_claim_ids.is_empty()
        } else {
            !covered_capacity_claim_ids.is_empty()
        }
}

fn exact_predecessor_state_preserved(
    predecessor_before: &SemanticEffectV1,
    predecessor_after: &SemanticEffectV1,
) -> bool {
    predecessor_before == predecessor_after
}

/// Validates a compensation as a new semantic effect. It can address a
/// retired/tombstoned predecessor, but it can never revive that predecessor's
/// lineage or generation.
pub fn assess_compensation_effect(
    link: &CompensationEffectLinkV1,
    predecessor_before: &SemanticEffectV1,
    predecessor_after: &SemanticEffectV1,
    compensation: &SemanticEffectV1,
    cause_observation: &ExternalEffectObservationV1,
    cause_finality: Option<&ExternalFinalityReceiptV1>,
    before_claims: &EffectConservationStateV1,
    after_claims: &EffectConservationStateV1,
    ledger: &ExternalEffectLedgerV1,
    current_compensation_generation_id: &str,
    tombstone: Option<&SemanticTombstone>,
) -> CompensationAssessmentV1 {
    if !link.structurally_valid()
        || !predecessor_before.structurally_valid()
        || !predecessor_after.structurally_valid()
        || !compensation.structurally_valid()
        || !cause_observation.structurally_valid()
        || !non_empty(current_compensation_generation_id)
    {
        return CompensationAssessmentV1::new(
            CompensationDispositionV1::InsufficientEvidence,
            &link.predecessor_effect_id,
            &link.compensation_effect_id,
        );
    }

    if link.predecessor_effect_id != predecessor_before.effect_id
        || link.predecessor_effect_id != predecessor_after.effect_id
        || link.predecessor_lineage_id != predecessor_before.lineage_id
        || link.predecessor_generation_id != predecessor_before.generation_id
        || link.predecessor_route_id != cause_observation.route_id
        || link.predecessor_provider_operation_id != cause_observation.provider_operation_id
        || link.cause_observation_id != cause_observation.observation_id
        || link.cause_provider_id != cause_observation.provider_id
        || link.cause_provider_profile_root != cause_observation.provider_profile_root
        || link.semantic_environment_root != predecessor_before.semantic_environment_root
        || link.semantic_environment_root != compensation.semantic_environment_root
    {
        return CompensationAssessmentV1::new(
            CompensationDispositionV1::BlockedCause,
            &link.predecessor_effect_id,
            &link.compensation_effect_id,
        );
    }

    if link.compensation_effect_id != compensation.effect_id
        || link.compensation_lineage_id != compensation.lineage_id
        || link.compensation_generation_id != compensation.generation_id
        || compensation.effect_id == predecessor_before.effect_id
        || compensation.lineage_id == predecessor_before.lineage_id
        || compensation.generation_id == predecessor_before.generation_id
        || compensation.tenant_id != predecessor_before.tenant_id
        || compensation.effect_id == link.predecessor_effect_id
    {
        return CompensationAssessmentV1::new(
            CompensationDispositionV1::BlockedIdentity,
            &link.predecessor_effect_id,
            &link.compensation_effect_id,
        );
    }

    if compensation.generation_id != current_compensation_generation_id {
        return CompensationAssessmentV1::new(
            CompensationDispositionV1::BlockedLifecycle,
            &link.predecessor_effect_id,
            &link.compensation_effect_id,
        );
    }

    if let Some(item) = tombstone {
        if item.lineage_id == compensation.lineage_id
            && item.retired_generation_id == compensation.generation_id
        {
            return CompensationAssessmentV1::new(
                CompensationDispositionV1::BlockedLifecycle,
                &link.predecessor_effect_id,
                &link.compensation_effect_id,
            );
        }
        if item.lineage_id == predecessor_before.lineage_id
            && item.retired_generation_id == predecessor_before.generation_id
            && compensation.lineage_id == predecessor_before.lineage_id
        {
            return CompensationAssessmentV1::new(
                CompensationDispositionV1::BlockedLifecycle,
                &link.predecessor_effect_id,
                &link.compensation_effect_id,
            );
        }
    }

    if !exact_predecessor_state_preserved(predecessor_before, predecessor_after) {
        return CompensationAssessmentV1::new(
            CompensationDispositionV1::BlockedMutation,
            &link.predecessor_effect_id,
            &link.compensation_effect_id,
        );
    }

    if let Some(finality) = cause_finality {
        if !finality.structurally_valid()
            || finality.effect_id != predecessor_before.effect_id
            || finality.observation_id != cause_observation.observation_id
            || finality.provider_id != cause_observation.provider_id
            || finality.provider_profile_root != cause_observation.provider_profile_root
            || finality.finality_state != ExternalFinalityStateV1::Applied
        {
            return CompensationAssessmentV1::new(
                CompensationDispositionV1::BlockedFinalityMismatch,
                &link.predecessor_effect_id,
                &link.compensation_effect_id,
            );
        }
        if link.cause_finality_receipt_id.as_deref() != Some(finality.receipt_id.as_str()) {
            return CompensationAssessmentV1::new(
                CompensationDispositionV1::BlockedFinalityMismatch,
                &link.predecessor_effect_id,
                &link.compensation_effect_id,
            );
        }
    } else if link.cause_finality_receipt_id.is_some() {
        return CompensationAssessmentV1::new(
            CompensationDispositionV1::BlockedFinalityMismatch,
            &link.predecessor_effect_id,
            &link.compensation_effect_id,
        );
    }

    match (link.reason, cause_observation.observed_state) {
        (
            CompensationReasonV1::Reversal
            | CompensationReasonV1::Refund
            | CompensationReasonV1::Correction,
            ExternalObservedStateV1::Applied,
        )
        | (
            CompensationReasonV1::Remediation,
            ExternalObservedStateV1::Applied
            | ExternalObservedStateV1::Reversed
            | ExternalObservedStateV1::Unknown
            | ExternalObservedStateV1::Contested,
        ) => {}
        (_, ExternalObservedStateV1::NotApplied) => {
            return CompensationAssessmentV1::new(
                CompensationDispositionV1::BlockedCause,
                &link.predecessor_effect_id,
                &link.compensation_effect_id,
            );
        }
        _ => {
            return CompensationAssessmentV1::new(
                CompensationDispositionV1::BlockedCause,
                &link.predecessor_effect_id,
                &link.compensation_effect_id,
            );
        }
    }

    if !conservation_allows_compensation(
        &link.predecessor_effect_id,
        &link.compensation_effect_id,
        &link.covered_capacity_claim_ids,
        before_claims,
        after_claims,
    ) {
        return CompensationAssessmentV1::new(
            CompensationDispositionV1::BlockedConservation,
            &link.predecessor_effect_id,
            &link.compensation_effect_id,
        );
    }

    if ledger.has_compensation_link(&link.link_id)
        || ledger.has_compensation_effect(&link.compensation_effect_id)
        || ledger.has_capacity_overlap(
            &link.predecessor_effect_id,
            &link.covered_capacity_claim_ids,
        )
    {
        return CompensationAssessmentV1::new(
            CompensationDispositionV1::BlockedDuplicate,
            &link.predecessor_effect_id,
            &link.compensation_effect_id,
        );
    }

    CompensationAssessmentV1::new(
        CompensationDispositionV1::Accepted,
        &link.predecessor_effect_id,
        &link.compensation_effect_id,
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExternalEffectLedgerDispositionV1 {
    Recorded,
    BlockedDuplicate,
    Conflict,
    InsufficientEvidence,
}

/// In-memory reference ledger. It retains evidence identity and refuses
/// duplicate/conflicting finality or overlapping compensation coverage.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalEffectLedgerV1 {
    pub observations: BTreeMap<String, ExternalEffectObservationV1>,
    pub finality_receipts: BTreeMap<String, ExternalFinalityReceiptV1>,
    pub finality_by_effect: BTreeMap<String, String>,
    pub compensation_links: BTreeMap<String, CompensationEffectLinkV1>,
}

impl ExternalEffectLedgerV1 {
    pub fn record_observation(
        &mut self,
        observation: ExternalEffectObservationV1,
    ) -> ExternalEffectLedgerDispositionV1 {
        if !observation.structurally_valid() {
            return ExternalEffectLedgerDispositionV1::InsufficientEvidence;
        }
        if let Some(existing) = self.observations.get(&observation.observation_id) {
            return if existing == &observation {
                ExternalEffectLedgerDispositionV1::BlockedDuplicate
            } else {
                ExternalEffectLedgerDispositionV1::Conflict
            };
        }
        self.observations
            .insert(observation.observation_id.clone(), observation);
        ExternalEffectLedgerDispositionV1::Recorded
    }

    pub fn record_finality(
        &mut self,
        receipt: ExternalFinalityReceiptV1,
    ) -> ExternalEffectLedgerDispositionV1 {
        if !receipt.structurally_valid() {
            return ExternalEffectLedgerDispositionV1::InsufficientEvidence;
        }
        if self.finality_receipts.contains_key(&receipt.receipt_id) {
            return ExternalEffectLedgerDispositionV1::BlockedDuplicate;
        }
        if let Some(existing_id) = self.finality_by_effect.get(&receipt.effect_id) {
            if existing_id == &receipt.receipt_id {
                return ExternalEffectLedgerDispositionV1::BlockedDuplicate;
            }
            return ExternalEffectLedgerDispositionV1::Conflict;
        }
        self.finality_by_effect
            .insert(receipt.effect_id.clone(), receipt.receipt_id.clone());
        self.finality_receipts
            .insert(receipt.receipt_id.clone(), receipt);
        ExternalEffectLedgerDispositionV1::Recorded
    }

    pub fn record_compensation(
        &mut self,
        link: CompensationEffectLinkV1,
    ) -> ExternalEffectLedgerDispositionV1 {
        if !link.structurally_valid() {
            return ExternalEffectLedgerDispositionV1::InsufficientEvidence;
        }
        if self.compensation_links.contains_key(&link.link_id)
            || self.has_compensation_effect(&link.compensation_effect_id)
        {
            return ExternalEffectLedgerDispositionV1::BlockedDuplicate;
        }
        if self.has_capacity_overlap(
            &link.predecessor_effect_id,
            &link.covered_capacity_claim_ids,
        ) {
            return ExternalEffectLedgerDispositionV1::Conflict;
        }
        self.compensation_links.insert(link.link_id.clone(), link);
        ExternalEffectLedgerDispositionV1::Recorded
    }

    pub fn has_compensation_link(&self, link_id: &str) -> bool {
        self.compensation_links.contains_key(link_id)
    }

    pub fn has_compensation_effect(&self, effect_id: &str) -> bool {
        self.compensation_links
            .values()
            .any(|item| item.compensation_effect_id == effect_id)
    }

    pub fn has_capacity_overlap(
        &self,
        predecessor_effect_id: &str,
        candidate_claims: &BTreeSet<String>,
    ) -> bool {
        self.compensation_links.values().any(|item| {
            item.predecessor_effect_id == predecessor_effect_id
                && !item
                    .covered_capacity_claim_ids
                    .is_disjoint(candidate_claims)
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArchiveFinalityUsePurposeV1 {
    HistoricalEvidence,
    CurrentExternalFinality,
    CurrentActuationAuthorization,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArchiveFinalityDispositionV1 {
    HistoricalOnly,
    BlockedCurrentFinality,
    BlockedCurrentActuation,
    BlockedArchiveBinding,
    InsufficientEvidence,
}

/// Archives can explain historical state but can never directly establish
/// current external finality, even when their source frontier equals the
/// current frontier.
pub fn assess_archive_finality_boundary(
    binding: &ArchiveRecoveryBindingV1,
    effect: &SemanticEffectV1,
    current_frontier_root: &str,
    purpose: ArchiveFinalityUsePurposeV1,
) -> ArchiveFinalityDispositionV1 {
    if !binding.structurally_valid()
        || !effect.structurally_valid()
        || !non_empty(current_frontier_root)
    {
        return ArchiveFinalityDispositionV1::InsufficientEvidence;
    }
    if binding.effect_id != effect.effect_id
        || binding.lifecycle_generation_id != effect.generation_id
        || binding.semantic_environment_root != effect.semantic_environment_root
    {
        return ArchiveFinalityDispositionV1::BlockedArchiveBinding;
    }
    match purpose {
        ArchiveFinalityUsePurposeV1::HistoricalEvidence => {
            ArchiveFinalityDispositionV1::HistoricalOnly
        }
        ArchiveFinalityUsePurposeV1::CurrentExternalFinality => {
            ArchiveFinalityDispositionV1::BlockedCurrentFinality
        }
        ArchiveFinalityUsePurposeV1::CurrentActuationAuthorization => {
            ArchiveFinalityDispositionV1::BlockedCurrentActuation
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArchiveFinalityWitnessV1 {
    pub archive_id: String,
    pub effect_id: String,
    pub source_frontier_root: String,
    pub current_frontier_root: String,
    pub claim_ceiling: String,
}

impl ArchiveFinalityWitnessV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.archive_id)
            && non_empty(&self.effect_id)
            && non_empty(&self.source_frontier_root)
            && non_empty(&self.current_frontier_root)
            && self.claim_ceiling == ARCHIVE_FINALITY_CLAIM_CEILING
    }
}

pub fn archive_finality_witness(
    binding: &ArchiveRecoveryBindingV1,
    current_frontier_root: &str,
) -> ArchiveFinalityWitnessV1 {
    ArchiveFinalityWitnessV1 {
        archive_id: binding.archive_id.clone(),
        effect_id: binding.effect_id.clone(),
        source_frontier_root: binding.source_frontier_root.clone(),
        current_frontier_root: current_frontier_root.to_owned(),
        claim_ceiling: ARCHIVE_FINALITY_CLAIM_CEILING.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::substitution_continuity::{
        IdempotencyScopeV1, ProviderOutcomeKindV1, OUTCOME_CLAIM_CEILING,
    };

    fn effect(id: &str, lineage: &str, generation: &str) -> SemanticEffectV1 {
        SemanticEffectV1 {
            effect_id: id.into(),
            lineage_id: lineage.into(),
            generation_id: generation.into(),
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

    fn substitution_profile(effect: &SemanticEffectV1) -> SemanticSubstitutionProfileV1 {
        SemanticSubstitutionProfileV1 {
            profile_id: "sub-profile-1".into(),
            semantic_environment_root: effect.semantic_environment_root.clone(),
            request_commitment: effect.request_commitment.clone(),
            effect_class: effect.effect_class.clone(),
            resource_id: effect.resource_id.clone(),
            tenant_id: effect.tenant_id.clone(),
            amount: effect.amount.clone(),
            unit: effect.unit.clone(),
            authority_claim_id: effect.authority_claim_id.clone(),
            consent_claim_id: effect.consent_claim_id.clone(),
            idempotency_key: effect.idempotency_key.clone(),
            idempotency_scope: IdempotencyScopeV1::ContractWide,
            idempotency_policy_root: Some("idem-policy-1".into()),
            allow_retry_after_no_effect: true,
            allow_same_provider_retry: true,
            allowed_provider_profile_roots: BTreeMap::from([
                ("provider-a".into(), "profile-a".into()),
                ("provider-b".into(), "profile-b".into()),
            ]),
            profile_commitment: "sub-profile-commitment".into(),
        }
    }

    fn route(
        effect: &SemanticEffectV1,
        provider: &str,
        provider_profile_root: &str,
        route_id: &str,
        operation_id: &str,
    ) -> ProviderRouteV1 {
        ProviderRouteV1 {
            route_id: route_id.into(),
            effect_id: effect.effect_id.clone(),
            substitution_profile_id: "sub-profile-1".into(),
            provider_id: provider.into(),
            provider_profile_root: provider_profile_root.into(),
            provider_operation_id: operation_id.into(),
            route_generation: 0,
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
            route_frontier_root: "frontier-1".into(),
        }
    }

    fn outcome(route: &ProviderRouteV1, kind: ProviderOutcomeKindV1) -> ProviderOutcomeV1 {
        ProviderOutcomeV1 {
            outcome_id: format!("outcome-{}", route.route_id),
            effect_id: route.effect_id.clone(),
            route_id: route.route_id.clone(),
            provider_operation_id: route.provider_operation_id.clone(),
            provider_id: route.provider_id.clone(),
            provider_profile_root: route.provider_profile_root.clone(),
            request_commitment: route.request_commitment.clone(),
            idempotency_key: route.idempotency_key.clone(),
            semantic_environment_root: route.semantic_environment_root.clone(),
            observed_frontier_root: route.route_frontier_root.clone(),
            outcome_kind: kind,
            evidence_root: "provider-evidence-1".into(),
            claim_ceiling: OUTCOME_CLAIM_CEILING.into(),
        }
    }

    fn observation(
        effect: &SemanticEffectV1,
        route: &ProviderRouteV1,
        outcome: &ProviderOutcomeV1,
        source: ExternalObservationSourceV1,
        state: ExternalObservedStateV1,
        frontier: &str,
    ) -> ExternalEffectObservationV1 {
        ExternalEffectObservationV1 {
            observation_id: format!("observation-{}", route.route_id),
            effect_id: effect.effect_id.clone(),
            effect_lineage_id: effect.lineage_id.clone(),
            lifecycle_generation_id: effect.generation_id.clone(),
            route_id: route.route_id.clone(),
            provider_id: route.provider_id.clone(),
            provider_operation_id: route.provider_operation_id.clone(),
            provider_profile_root: route.provider_profile_root.clone(),
            provider_outcome_id: outcome.outcome_id.clone(),
            request_commitment: effect.request_commitment.clone(),
            idempotency_key: effect.idempotency_key.clone(),
            semantic_environment_root: effect.semantic_environment_root.clone(),
            observed_frontier_root: frontier.into(),
            observed_state: state,
            source,
            evidence_root: "independent-evidence-1".into(),
            claim_ceiling: EXTERNAL_FINALITY_CLAIM_CEILING.into(),
        }
    }

    fn finality_profile(
        effect: &SemanticEffectV1,
        state: ExternalFinalityStateV1,
        current_frontier_required: bool,
    ) -> ExternalFinalityProfileV1 {
        ExternalFinalityProfileV1 {
            profile_id: "finality-profile-1".into(),
            effect_id: effect.effect_id.clone(),
            substitution_profile_id: "sub-profile-1".into(),
            semantic_environment_root: effect.semantic_environment_root.clone(),
            request_commitment: effect.request_commitment.clone(),
            idempotency_key: effect.idempotency_key.clone(),
            allowed_observation_sources: BTreeSet::from([
                ExternalObservationSourceV1::IndependentObserver,
                ExternalObservationSourceV1::SettlementAuthority,
                ExternalObservationSourceV1::QualifiedIndependentReconciliation,
            ]),
            required_finality_state: state,
            current_frontier_required,
            profile_commitment: "finality-profile-commitment".into(),
            claim_ceiling: EXTERNAL_FINALITY_CLAIM_CEILING.into(),
        }
    }

    fn finality_receipt(
        effect: &SemanticEffectV1,
        route: &ProviderRouteV1,
        outcome: &ProviderOutcomeV1,
        observation: &ExternalEffectObservationV1,
        profile: &ExternalFinalityProfileV1,
    ) -> ExternalFinalityReceiptV1 {
        ExternalFinalityReceiptV1 {
            receipt_id: "finality-1".into(),
            effect_id: effect.effect_id.clone(),
            effect_lineage_id: effect.lineage_id.clone(),
            lifecycle_generation_id: effect.generation_id.clone(),
            route_id: route.route_id.clone(),
            provider_id: route.provider_id.clone(),
            provider_operation_id: route.provider_operation_id.clone(),
            provider_profile_root: route.provider_profile_root.clone(),
            provider_outcome_id: outcome.outcome_id.clone(),
            observation_id: observation.observation_id.clone(),
            observation_frontier_root: observation.observed_frontier_root.clone(),
            finality_profile_id: profile.profile_id.clone(),
            semantic_environment_root: effect.semantic_environment_root.clone(),
            finality_state: profile.required_finality_state,
            evidence_root: "finality-evidence-1".into(),
            finality_commitment: "finality-commitment-1".into(),
            claim_ceiling: EXTERNAL_FINALITY_CLAIM_CEILING.into(),
        }
    }

    fn tombstone(effect: &SemanticEffectV1) -> SemanticTombstone {
        SemanticTombstone {
            tombstone_id: "tombstone-1".into(),
            lineage_id: effect.lineage_id.clone(),
            retired_generation_id: effect.generation_id.clone(),
            retired_creation_event_id: "creation-1".into(),
            causal_frontier_root: "frontier-0".into(),
            reason: "Revoked".into(),
            provenance_root: "provenance-1".into(),
        }
    }

    fn compensation_effect(predecessor: &SemanticEffectV1) -> SemanticEffectV1 {
        SemanticEffectV1 {
            effect_id: "effect-comp".into(),
            lineage_id: "lineage-comp".into(),
            generation_id: "generation-comp".into(),
            request_commitment: "comp-request".into(),
            semantic_environment_root: predecessor.semantic_environment_root.clone(),
            effect_class: "refund".into(),
            resource_id: "resource-refund".into(),
            tenant_id: predecessor.tenant_id.clone(),
            amount: "10".into(),
            unit: "unit".into(),
            authority_claim_id: "authority-comp".into(),
            consent_claim_id: "consent-comp".into(),
            idempotency_key: "idem-comp".into(),
        }
    }

    fn compensation_link(
        predecessor: &SemanticEffectV1,
        compensation: &SemanticEffectV1,
        observation: &ExternalEffectObservationV1,
        finality: Option<&ExternalFinalityReceiptV1>,
    ) -> CompensationEffectLinkV1 {
        CompensationEffectLinkV1 {
            link_id: "comp-link-1".into(),
            predecessor_effect_id: predecessor.effect_id.clone(),
            predecessor_lineage_id: predecessor.lineage_id.clone(),
            predecessor_generation_id: predecessor.generation_id.clone(),
            predecessor_route_id: observation.route_id.clone(),
            predecessor_provider_operation_id: observation.provider_operation_id.clone(),
            compensation_effect_id: compensation.effect_id.clone(),
            compensation_lineage_id: compensation.lineage_id.clone(),
            compensation_generation_id: compensation.generation_id.clone(),
            cause_observation_id: observation.observation_id.clone(),
            cause_finality_receipt_id: finality.map(|item| item.receipt_id.clone()),
            cause_provider_id: observation.provider_id.clone(),
            cause_provider_profile_root: observation.provider_profile_root.clone(),
            reason: CompensationReasonV1::Refund,
            semantic_environment_root: predecessor.semantic_environment_root.clone(),
            covered_capacity_claim_ids: BTreeSet::from(["capacity-1".into()]),
            accounting_root: "accounting-1".into(),
            link_commitment: "comp-link-commitment".into(),
            claim_ceiling: COMPENSATION_CLAIM_CEILING.into(),
        }
    }

    fn claims(predecessor: &SemanticEffectV1, compensation: &SemanticEffectV1) -> (EffectConservationStateV1, EffectConservationStateV1) {
        let before = EffectConservationStateV1 {
            effect_ids: BTreeSet::from([predecessor.effect_id.clone()]),
            authority_claim_ids: BTreeSet::from([predecessor.authority_claim_id.clone()]),
            capacity_claim_ids: BTreeSet::from(["capacity-1".into()]),
            consent_claim_ids: BTreeSet::from([predecessor.consent_claim_id.clone()]),
        };
        let after = EffectConservationStateV1 {
            effect_ids: BTreeSet::from([predecessor.effect_id.clone(), compensation.effect_id.clone()]),
            authority_claim_ids: BTreeSet::from([
                predecessor.authority_claim_id.clone(),
                compensation.authority_claim_id.clone(),
            ]),
            capacity_claim_ids: BTreeSet::from(["capacity-1".into()]),
            consent_claim_ids: BTreeSet::from([
                predecessor.consent_claim_id.clone(),
                compensation.consent_claim_id.clone(),
            ]),
        };
        (before, after)
    }

    fn archive_binding(effect: &SemanticEffectV1) -> ArchiveRecoveryBindingV1 {
        ArchiveRecoveryBindingV1 {
            archive_id: "archive-1".into(),
            source_snapshot_root: "snapshot-1".into(),
            source_frontier_root: "frontier-1".into(),
            membership_epoch: 1,
            semantic_environment_root: effect.semantic_environment_root.clone(),
            historical_profile_id: "history-profile-1".into(),
            effect_id: effect.effect_id.clone(),
            request_commitment: effect.request_commitment.clone(),
            lifecycle_generation_id: effect.generation_id.clone(),
            reconstructed_state_root: "state-1".into(),
            evidence_root: "archive-evidence-1".into(),
            claim_ceiling: crate::substitution_continuity::RECOVERY_INPUT_CLAIM_CEILING.into(),
        }
    }

    #[test]
    fn provider_success_without_independent_finality_is_not_final() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let sub = substitution_profile(&effect);
        let route = route(&effect, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &effect,
            &route,
            &outcome,
            ExternalObservationSourceV1::ProviderReported,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let profile = finality_profile(&effect, ExternalFinalityStateV1::Applied, true);
        let receipt = finality_receipt(&effect, &route, &outcome, &observation, &profile);

        assert_eq!(
            assess_external_finality(
                &effect,
                &sub,
                &profile,
                &route,
                &outcome,
                &observation,
                &receipt,
                "frontier-1",
                FinalityUsePurposeV1::CurrentFinality,
                None,
            ),
            FinalityDispositionV1::BlockedProviderReported
        );
    }

    #[test]
    fn independent_finality_is_accepted_at_current_frontier() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let sub = substitution_profile(&effect);
        let route = route(&effect, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &effect,
            &route,
            &outcome,
            ExternalObservationSourceV1::SettlementAuthority,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let profile = finality_profile(&effect, ExternalFinalityStateV1::Applied, true);
        let receipt = finality_receipt(&effect, &route, &outcome, &observation, &profile);

        assert_eq!(
            assess_external_finality(
                &effect,
                &sub,
                &profile,
                &route,
                &outcome,
                &observation,
                &receipt,
                "frontier-1",
                FinalityUsePurposeV1::CurrentFinality,
                None,
            ),
            FinalityDispositionV1::AcceptedCurrent
        );
    }

    #[test]
    fn stale_observation_is_rejected_for_current_finality() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let sub = substitution_profile(&effect);
        let route = route(&effect, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &effect,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-old",
        );
        let profile = finality_profile(&effect, ExternalFinalityStateV1::Applied, true);
        let receipt = finality_receipt(&effect, &route, &outcome, &observation, &profile);

        assert_eq!(
            assess_external_finality(
                &effect,
                &sub,
                &profile,
                &route,
                &outcome,
                &observation,
                &receipt,
                "frontier-current",
                FinalityUsePurposeV1::CurrentFinality,
                None,
            ),
            FinalityDispositionV1::BlockedStaleObservation
        );
    }

    #[test]
    fn wrong_route_or_operation_cannot_supply_finality() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let sub = substitution_profile(&effect);
        let route_a = route(&effect, "provider-a", "profile-a", "route-a", "operation-a");
        let route_b = route(&effect, "provider-b", "profile-b", "route-b", "operation-b");
        let outcome_a = outcome(&route_a, ProviderOutcomeKindV1::Succeeded);
        let observation_a = observation(
            &effect,
            &route_a,
            &outcome_a,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let profile = finality_profile(&effect, ExternalFinalityStateV1::Applied, true);
        let receipt = finality_receipt(&effect, &route_b, &outcome_a, &observation_a, &profile);

        assert_eq!(
            assess_external_finality(
                &effect,
                &sub,
                &profile,
                &route_b,
                &outcome_a,
                &observation_a,
                &receipt,
                "frontier-1",
                FinalityUsePurposeV1::CurrentFinality,
                None,
            ),
            FinalityDispositionV1::BlockedReceiptMismatch
        );
    }

    #[test]
    fn finality_profile_drift_is_rejected() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let sub = substitution_profile(&effect);
        let route = route(&effect, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &effect,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let mut profile = finality_profile(&effect, ExternalFinalityStateV1::Applied, true);
        profile.semantic_environment_root = "other-env".into();
        let receipt = finality_receipt(&effect, &route, &outcome, &observation, &profile);

        assert_eq!(
            assess_external_finality(
                &effect,
                &sub,
                &profile,
                &route,
                &outcome,
                &observation,
                &receipt,
                "frontier-1",
                FinalityUsePurposeV1::CurrentFinality,
                None,
            ),
            FinalityDispositionV1::BlockedProfile
        );
    }

    #[test]
    fn unknown_external_state_stays_unresolved() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let sub = substitution_profile(&effect);
        let route = route(&effect, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Unknown);
        let observation = observation(
            &effect,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Unknown,
            "frontier-1",
        );
        let profile = finality_profile(&effect, ExternalFinalityStateV1::Applied, true);
        let receipt = finality_receipt(&effect, &route, &outcome, &observation, &profile);

        assert_eq!(
            assess_external_finality(
                &effect,
                &sub,
                &profile,
                &route,
                &outcome,
                &observation,
                &receipt,
                "frontier-1",
                FinalityUsePurposeV1::CurrentFinality,
                None,
            ),
            FinalityDispositionV1::BlockedUnresolved
        );
    }

    #[test]
    fn finality_cannot_be_used_as_actuation_authorization() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let sub = substitution_profile(&effect);
        let route = route(&effect, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &effect,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let profile = finality_profile(&effect, ExternalFinalityStateV1::Applied, true);
        let receipt = finality_receipt(&effect, &route, &outcome, &observation, &profile);

        assert_eq!(
            assess_external_finality(
                &effect,
                &sub,
                &profile,
                &route,
                &outcome,
                &observation,
                &receipt,
                "frontier-1",
                FinalityUsePurposeV1::CurrentActuationAuthorization,
                None,
            ),
            FinalityDispositionV1::BlockedCurrentAuthorization
        );
    }

    #[test]
    fn tombstoned_effect_cannot_gain_finality() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let sub = substitution_profile(&effect);
        let route = route(&effect, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &effect,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let profile = finality_profile(&effect, ExternalFinalityStateV1::Applied, true);
        let receipt = finality_receipt(&effect, &route, &outcome, &observation, &profile);
        let tombstone = tombstone(&effect);

        assert_eq!(
            assess_external_finality(
                &effect,
                &sub,
                &profile,
                &route,
                &outcome,
                &observation,
                &receipt,
                "frontier-1",
                FinalityUsePurposeV1::CurrentFinality,
                Some(&tombstone),
            ),
            FinalityDispositionV1::BlockedLifecycle
        );
    }

    #[test]
    fn compensation_is_a_distinct_effect_and_preserves_predecessor() {
        let predecessor = effect("effect-1", "lineage-1", "generation-1");
        let compensation = compensation_effect(&predecessor);
        let sub = substitution_profile(&predecessor);
        let route = route(&predecessor, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &predecessor,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let profile = finality_profile(&predecessor, ExternalFinalityStateV1::Applied, true);
        let finality = finality_receipt(&predecessor, &route, &outcome, &observation, &profile);
        let link = compensation_link(&predecessor, &compensation, &observation, Some(&finality));
        let (before, after) = claims(&predecessor, &compensation);
        let assessment = assess_compensation_effect(
            &link,
            &predecessor,
            &predecessor,
            &compensation,
            &observation,
            Some(&finality),
            &before,
            &after,
            &ExternalEffectLedgerV1::default(),
            "generation-comp",
            None,
        );

        assert_eq!(assessment.disposition, CompensationDispositionV1::Accepted);
        assert_ne!(predecessor.effect_id, compensation.effect_id);
        assert_eq!(predecessor, predecessor);
    }

    #[test]
    fn compensation_without_explicit_cause_is_rejected() {
        let predecessor = effect("effect-1", "lineage-1", "generation-1");
        let compensation = compensation_effect(&predecessor);
        let route = route(&predecessor, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &predecessor,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let mut link = compensation_link(&predecessor, &compensation, &observation, None);
        let (before, after) = claims(&predecessor, &compensation);
        link.cause_observation_id.clear();

        assert_eq!(
            assess_compensation_effect(
                &link,
                &predecessor,
                &predecessor,
                &compensation,
                &observation,
                None,
                &before,
                &after,
                &ExternalEffectLedgerV1::default(),
                "generation-comp",
                None,
            )
            .disposition,
            CompensationDispositionV1::InsufficientEvidence
        );

        let mut mismatched = compensation_link(&predecessor, &compensation, &observation, None);
        mismatched.cause_observation_id = "different".into();
        assert_eq!(
            assess_compensation_effect(
                &mismatched,
                &predecessor,
                &predecessor,
                &compensation,
                &observation,
                None,
                &before,
                &after,
                &ExternalEffectLedgerV1::default(),
                "generation-comp",
                None,
            )
            .disposition,
            CompensationDispositionV1::BlockedCause
        );
    }

    #[test]
    fn compensation_mutation_of_predecessor_is_rejected() {
        let predecessor = effect("effect-1", "lineage-1", "generation-1");
        let compensation = compensation_effect(&predecessor);
        let route = route(&predecessor, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &predecessor,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let link = compensation_link(&predecessor, &compensation, &observation, None);
        let mut mutated = predecessor.clone();
        mutated.amount = "11".into();
        let (before, after) = claims(&predecessor, &compensation);

        assert_eq!(
            assess_compensation_effect(
                &link,
                &predecessor,
                &mutated,
                &compensation,
                &observation,
                None,
                &before,
                &after,
                &ExternalEffectLedgerV1::default(),
                "generation-comp",
                None,
            )
            .disposition,
            CompensationDispositionV1::BlockedMutation
        );
    }

    #[test]
    fn compensation_double_counting_is_rejected() {
        let predecessor = effect("effect-1", "lineage-1", "generation-1");
        let compensation = compensation_effect(&predecessor);
        let route = route(&predecessor, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &predecessor,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let link = compensation_link(&predecessor, &compensation, &observation, None);
        let (before, after) = claims(&predecessor, &compensation);
        let mut ledger = ExternalEffectLedgerV1::default();
        assert_eq!(
            ledger.record_compensation(link.clone()),
            ExternalEffectLedgerDispositionV1::Recorded
        );

        let mut second = compensation_effect(&predecessor);
        second.effect_id = "effect-comp-2".into();
        second.lineage_id = "lineage-comp-2".into();
        second.generation_id = "generation-comp-2".into();
        let second_link = compensation_link(&predecessor, &second, &observation, None);

        assert_eq!(
            assess_compensation_effect(
                &second_link,
                &predecessor,
                &predecessor,
                &second,
                &observation,
                None,
                &before,
                &{
                    let mut next = after.clone();
                    next.effect_ids.insert(second.effect_id.clone());
                    next
                },
                &ledger,
                "generation-comp-2",
                None,
            )
            .disposition,
            CompensationDispositionV1::BlockedDuplicate
        );
    }

    #[test]
    fn tombstoned_predecessor_can_be_compensated_only_as_new_lineage() {
        let predecessor = effect("effect-1", "lineage-1", "generation-1");
        let compensation = compensation_effect(&predecessor);
        let route = route(&predecessor, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &predecessor,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let link = compensation_link(&predecessor, &compensation, &observation, None);
        let (before, after) = claims(&predecessor, &compensation);
        let tombstone = tombstone(&predecessor);

        assert_eq!(
            assess_compensation_effect(
                &link,
                &predecessor,
                &predecessor,
                &compensation,
                &observation,
                None,
                &before,
                &after,
                &ExternalEffectLedgerV1::default(),
                "generation-comp",
                Some(&tombstone),
            )
            .disposition,
            CompensationDispositionV1::Accepted
        );

        let mut revived = compensation.clone();
        revived.effect_id = predecessor.effect_id.clone();
        let mut revived_link = link.clone();
        revived_link.compensation_effect_id = revived.effect_id.clone();

        assert_eq!(
            assess_compensation_effect(
                &revived_link,
                &predecessor,
                &predecessor,
                &revived,
                &observation,
                None,
                &before,
                &before,
                &ExternalEffectLedgerV1::default(),
                "generation-1",
                Some(&tombstone),
            )
            .disposition,
            CompensationDispositionV1::BlockedIdentity
        );
    }

    #[test]
    fn archive_cannot_establish_current_finality_even_at_same_frontier() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let binding = archive_binding(&effect);

        assert_eq!(
            assess_archive_finality_boundary(
                &binding,
                &effect,
                "frontier-1",
                ArchiveFinalityUsePurposeV1::HistoricalEvidence,
            ),
            ArchiveFinalityDispositionV1::HistoricalOnly
        );
        assert_eq!(
            assess_archive_finality_boundary(
                &binding,
                &effect,
                "frontier-1",
                ArchiveFinalityUsePurposeV1::CurrentExternalFinality,
            ),
            ArchiveFinalityDispositionV1::BlockedCurrentFinality
        );
    }

    #[test]
    fn archive_with_wrong_effect_is_not_reusable_for_finality() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let other = effect("effect-2", "lineage-2", "generation-2");
        let binding = archive_binding(&effect);

        assert_eq!(
            assess_archive_finality_boundary(
                &binding,
                &other,
                "frontier-current",
                ArchiveFinalityUsePurposeV1::CurrentExternalFinality,
            ),
            ArchiveFinalityDispositionV1::BlockedArchiveBinding
        );
    }

    #[test]
    fn same_visible_state_does_not_merge_effect_lineages() {
        let first = effect("effect-1", "lineage-1", "generation-1");
        let second = effect("effect-2", "lineage-2", "generation-2");
        let sub_first = substitution_profile(&first);
        let sub_second = substitution_profile(&second);
        let route_first = route(&first, "provider-a", "profile-a", "route-a", "operation-a");
        let route_second = route(&second, "provider-a", "profile-a", "route-b", "operation-b");
        let outcome_first = outcome(&route_first, ProviderOutcomeKindV1::Succeeded);
        let outcome_second = outcome(&route_second, ProviderOutcomeKindV1::Succeeded);
        let observation_first = observation(
            &first,
            &route_first,
            &outcome_first,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let observation_second = observation(
            &second,
            &route_second,
            &outcome_second,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let profile_first = finality_profile(&first, ExternalFinalityStateV1::Applied, true);
        let receipt_first =
            finality_receipt(&first, &route_first, &outcome_first, &observation_first, &profile_first);

        assert_eq!(
            assess_external_finality(
                &second,
                &sub_second,
                &profile_first,
                &route_second,
                &outcome_second,
                &observation_second,
                &receipt_first,
                "frontier-1",
                FinalityUsePurposeV1::CurrentFinality,
                None,
            ),
            FinalityDispositionV1::BlockedProfile
        );
        let _ = sub_first;
    }

    #[test]
    fn ledger_rejects_conflicting_finality_for_one_effect() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let sub = substitution_profile(&effect);
        let route = route(&effect, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &effect,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let profile = finality_profile(&effect, ExternalFinalityStateV1::Applied, true);
        let receipt = finality_receipt(&effect, &route, &outcome, &observation, &profile);
        let mut ledger = ExternalEffectLedgerV1::default();

        assert_eq!(
            assess_external_finality(
                &effect,
                &sub,
                &profile,
                &route,
                &outcome,
                &observation,
                &receipt,
                "frontier-1",
                FinalityUsePurposeV1::CurrentFinality,
                None,
            ),
            FinalityDispositionV1::AcceptedCurrent
        );
        assert_eq!(
            ledger.record_finality(receipt.clone()),
            ExternalEffectLedgerDispositionV1::Recorded
        );

        let mut conflicting = receipt;
        conflicting.receipt_id = "finality-2".into();
        conflicting.finality_state = ExternalFinalityStateV1::NotApplied;
        assert_eq!(
            ledger.record_finality(conflicting),
            ExternalEffectLedgerDispositionV1::Conflict
        );
    }

    #[test]
    fn finality_receipt_does_not_create_a_new_effect() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let route = route(&effect, "provider-a", "profile-a", "route-a", "operation-a");
        let outcome = outcome(&route, ProviderOutcomeKindV1::Succeeded);
        let observation = observation(
            &effect,
            &route,
            &outcome,
            ExternalObservationSourceV1::IndependentObserver,
            ExternalObservedStateV1::Applied,
            "frontier-1",
        );
        let profile = finality_profile(&effect, ExternalFinalityStateV1::Applied, true);
        let receipt = finality_receipt(&effect, &route, &outcome, &observation, &profile);

        assert_eq!(receipt.effect_id, effect.effect_id);
        assert_ne!(receipt.receipt_id, "effect-2");
        assert!(receipt.structurally_valid());
    }

    #[test]
    fn archive_finality_witness_is_claim_bounded() {
        let effect = effect("effect-1", "lineage-1", "generation-1");
        let binding = archive_binding(&effect);
        let witness = archive_finality_witness(&binding, "frontier-current");

        assert!(witness.structurally_valid());
        assert_eq!(witness.claim_ceiling, ARCHIVE_FINALITY_CLAIM_CEILING);
    }
}
