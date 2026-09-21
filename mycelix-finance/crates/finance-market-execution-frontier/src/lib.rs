#![forbid(unsafe_code)]

//! FIN-MKT-002B0: resolve provider fill observations into unique observed economic executions.
//!
//! This crate is deliberately narrower than order-state projection. It validates one exact
//! market-order observation scope, de-duplicates observation envelopes, groups fills by the
//! provider execution identity admitted by that scope, resolves correction/bust aliases, and
//! returns the effective observed execution frontier. It establishes no provider currentness,
//! frontier completeness, position ownership, authority, or settlement.

use mycelix_finance_market_core::{
    CanonicalMarketOrderIntentV1, Digest32, MarketExternalIdV1, MarketPriceV1,
    MarketProfileRefV1, MarketQuantityV1,
};
use mycelix_finance_market_observation::{
    validate_subject_against_intent_v1, CanonicalFillAdjustmentObservationV1,
    CanonicalFillObservationV1, FillAdjustmentKindV1, MarketOrderObservationSubjectV1,
};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const MAX_EXECUTION_FRONTIER_ITEMS_V1: usize = 4096;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExecutionFrontierError {
    FrontierTooLarge,
    ObservationSubjectMismatch,
    ProviderProfileMismatch,
    ObservationProfileMismatch,
    ProviderOrderRefConflict,
    FillObservationIdentityConflict,
    FillSemanticAliasConflict,
    ExecutionIdentityConflict,
    AdjustmentObservationIdentityConflict,
    AdjustmentTopologyConflict,
    AdjustmentCycle,
}

impl fmt::Display for ExecutionFrontierError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::FrontierTooLarge => "market execution frontier exceeds the V1 bound",
            Self::ObservationSubjectMismatch => {
                "observation subject is outside the selected immutable order lineage"
            }
            Self::ProviderProfileMismatch => {
                "observation provider profile does not match the execution-frontier scope"
            }
            Self::ObservationProfileMismatch => {
                "fill or adjustment observation profile does not match its scope role"
            }
            Self::ProviderOrderRefConflict => {
                "multiple incompatible provider-order refs occur in one immutable intent frontier"
            }
            Self::FillObservationIdentityConflict => {
                "one fill observation identity is reused for different normalized semantics"
            }
            Self::FillSemanticAliasConflict => {
                "one fill semantic commitment resolves to incompatible execution identities"
            }
            Self::ExecutionIdentityConflict => {
                "one provider execution identity is reused for incompatible economic execution semantics"
            }
            Self::AdjustmentObservationIdentityConflict => {
                "one adjustment observation identity is reused for different normalized semantics"
            }
            Self::AdjustmentTopologyConflict => {
                "fill correction/bust topology conflicts after execution alias resolution"
            }
            Self::AdjustmentCycle => "fill correction topology contains a cycle",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ExecutionFrontierError {}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ExecutionFrontierScopeV1 {
    pub provider_profile: MarketProfileRefV1,
    pub fill_observation_profile: MarketProfileRefV1,
    pub adjustment_observation_profile: MarketProfileRefV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct EffectiveObservedExecutionV1 {
    pub provider_execution_ref: MarketExternalIdV1,
    pub executed_quantity: MarketQuantityV1,
    pub execution_price: MarketPriceV1,
    pub venue_ref: Option<MarketExternalIdV1>,
    pub supporting_fill_semantic_commitments: Vec<Digest32>,
    pub supporting_observation_count: u32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub enum ExecutionFrontierResolutionV1 {
    Resolved {
        had_adjustments: bool,
        effective_executions: Vec<EffectiveObservedExecutionV1>,
    },
    AdjustmentResolutionIndeterminate,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ObservedExecutionFrontierV1 {
    pub intent_commitment: Digest32,
    pub scope: ExecutionFrontierScopeV1,
    pub provider_order_ref_observed: Option<MarketExternalIdV1>,
    pub unique_fill_observation_count: u32,
    pub unique_execution_count: u32,
    pub unique_adjustment_count: u32,
    pub resolution: ExecutionFrontierResolutionV1,
}

struct ExecutionRecord {
    provider_execution_ref: MarketExternalIdV1,
    executed_quantity: MarketQuantityV1,
    execution_price: MarketPriceV1,
    venue_ref: Option<MarketExternalIdV1>,
    fill_semantic_aliases: BTreeSet<Digest32>,
}

impl ExecutionRecord {
    fn absorb_fill(
        &mut self,
        fill: &CanonicalFillObservationV1,
    ) -> Result<(), ExecutionFrontierError> {
        if self.executed_quantity != fill.input().executed_quantity
            || self.execution_price != fill.input().execution_price
        {
            return Err(ExecutionFrontierError::ExecutionIdentityConflict);
        }

        match (&self.venue_ref, &fill.input().venue_ref) {
            (None, Some(candidate)) => self.venue_ref = Some(candidate.clone()),
            (Some(existing), Some(candidate)) if existing != candidate => {
                return Err(ExecutionFrontierError::ExecutionIdentityConflict);
            }
            _ => {}
        }

        self.fill_semantic_aliases
            .insert(fill.semantic_commitment());
        Ok(())
    }

    fn into_effective(self) -> Result<EffectiveObservedExecutionV1, ExecutionFrontierError> {
        let supporting_observation_count = u32::try_from(self.fill_semantic_aliases.len())
            .map_err(|_| ExecutionFrontierError::FrontierTooLarge)?;
        Ok(EffectiveObservedExecutionV1 {
            provider_execution_ref: self.provider_execution_ref,
            executed_quantity: self.executed_quantity,
            execution_price: self.execution_price,
            venue_ref: self.venue_ref,
            supporting_fill_semantic_commitments: self.fill_semantic_aliases.into_iter().collect(),
            supporting_observation_count,
        })
    }
}

#[derive(Clone)]
enum AdjustmentEdge {
    Correction(MarketExternalIdV1),
    Bust,
}

pub fn resolve_observed_execution_frontier_v1(
    intent: &CanonicalMarketOrderIntentV1,
    scope: &ExecutionFrontierScopeV1,
    fills: &[CanonicalFillObservationV1],
    adjustments: &[CanonicalFillAdjustmentObservationV1],
) -> Result<ObservedExecutionFrontierV1, ExecutionFrontierError> {
    let total = fills
        .len()
        .checked_add(adjustments.len())
        .ok_or(ExecutionFrontierError::FrontierTooLarge)?;
    if total > MAX_EXECUTION_FRONTIER_ITEMS_V1 {
        return Err(ExecutionFrontierError::FrontierTooLarge);
    }

    let provider_order_ref_observed = validate_scope(intent, scope, fills, adjustments)?;
    let unique_fills = deduplicate_fill_observations(fills)?;
    let unique_adjustments = deduplicate_adjustment_observations(adjustments)?;

    let (executions, semantic_aliases) = resolve_execution_identities(&unique_fills)?;
    let unique_execution_count = u32::try_from(executions.len())
        .map_err(|_| ExecutionFrontierError::FrontierTooLarge)?;

    let resolution = resolve_adjustments(executions, &semantic_aliases, &unique_adjustments)?;

    Ok(ObservedExecutionFrontierV1 {
        intent_commitment: intent.commitment(),
        scope: scope.clone(),
        provider_order_ref_observed,
        unique_fill_observation_count: u32::try_from(unique_fills.len())
            .map_err(|_| ExecutionFrontierError::FrontierTooLarge)?,
        unique_execution_count,
        unique_adjustment_count: u32::try_from(unique_adjustments.len())
            .map_err(|_| ExecutionFrontierError::FrontierTooLarge)?,
        resolution,
    })
}

fn validate_scope(
    intent: &CanonicalMarketOrderIntentV1,
    scope: &ExecutionFrontierScopeV1,
    fills: &[CanonicalFillObservationV1],
    adjustments: &[CanonicalFillAdjustmentObservationV1],
) -> Result<Option<MarketExternalIdV1>, ExecutionFrontierError> {
    let mut provider_order_ref: Option<MarketExternalIdV1> = None;

    for fill in fills {
        validate_subject_scope(
            &fill.input().subject,
            intent,
            &scope.provider_profile,
            &mut provider_order_ref,
        )?;
        if fill.input().observation_ref.observation_profile != scope.fill_observation_profile {
            return Err(ExecutionFrontierError::ObservationProfileMismatch);
        }
    }

    for adjustment in adjustments {
        validate_subject_scope(
            &adjustment.input().subject,
            intent,
            &scope.provider_profile,
            &mut provider_order_ref,
        )?;
        if adjustment.input().observation_ref.observation_profile
            != scope.adjustment_observation_profile
        {
            return Err(ExecutionFrontierError::ObservationProfileMismatch);
        }
    }

    Ok(provider_order_ref)
}

fn validate_subject_scope(
    subject: &MarketOrderObservationSubjectV1,
    intent: &CanonicalMarketOrderIntentV1,
    provider_profile: &MarketProfileRefV1,
    provider_order_ref: &mut Option<MarketExternalIdV1>,
) -> Result<(), ExecutionFrontierError> {
    validate_subject_against_intent_v1(subject, intent)
        .map_err(|_| ExecutionFrontierError::ObservationSubjectMismatch)?;

    if &subject.provider_profile != provider_profile {
        return Err(ExecutionFrontierError::ProviderProfileMismatch);
    }

    if let Some(candidate) = subject.provider_order_ref.as_ref() {
        match provider_order_ref.as_ref() {
            None => *provider_order_ref = Some(candidate.clone()),
            Some(existing) if existing == candidate => {}
            Some(_) => return Err(ExecutionFrontierError::ProviderOrderRefConflict),
        }
    }

    Ok(())
}

fn deduplicate_fill_observations(
    fills: &[CanonicalFillObservationV1],
) -> Result<Vec<&CanonicalFillObservationV1>, ExecutionFrontierError> {
    let mut by_identity: BTreeMap<Digest32, &CanonicalFillObservationV1> = BTreeMap::new();

    for fill in fills {
        match by_identity.get(&fill.identity_commitment()) {
            None => {
                by_identity.insert(fill.identity_commitment(), fill);
            }
            Some(existing) if existing.semantic_commitment() == fill.semantic_commitment() => {}
            Some(_) => return Err(ExecutionFrontierError::FillObservationIdentityConflict),
        }
    }

    Ok(by_identity.into_values().collect())
}

fn deduplicate_adjustment_observations(
    adjustments: &[CanonicalFillAdjustmentObservationV1],
) -> Result<Vec<&CanonicalFillAdjustmentObservationV1>, ExecutionFrontierError> {
    let mut by_identity: BTreeMap<Digest32, &CanonicalFillAdjustmentObservationV1> =
        BTreeMap::new();

    for adjustment in adjustments {
        match by_identity.get(&adjustment.identity_commitment()) {
            None => {
                by_identity.insert(adjustment.identity_commitment(), adjustment);
            }
            Some(existing)
                if existing.semantic_commitment() == adjustment.semantic_commitment() => {}
            Some(_) => {
                return Err(ExecutionFrontierError::AdjustmentObservationIdentityConflict)
            }
        }
    }

    Ok(by_identity.into_values().collect())
}

fn resolve_execution_identities(
    fills: &[&CanonicalFillObservationV1],
) -> Result<
    (
        BTreeMap<MarketExternalIdV1, ExecutionRecord>,
        BTreeMap<Digest32, MarketExternalIdV1>,
    ),
    ExecutionFrontierError,
> {
    let mut executions: BTreeMap<MarketExternalIdV1, ExecutionRecord> = BTreeMap::new();
    let mut semantic_aliases: BTreeMap<Digest32, MarketExternalIdV1> = BTreeMap::new();

    for fill in fills {
        let execution_ref = fill.input().provider_execution_ref.clone();

        match executions.get_mut(&execution_ref) {
            None => {
                let mut aliases = BTreeSet::new();
                aliases.insert(fill.semantic_commitment());
                executions.insert(
                    execution_ref.clone(),
                    ExecutionRecord {
                        provider_execution_ref: execution_ref.clone(),
                        executed_quantity: fill.input().executed_quantity.clone(),
                        execution_price: fill.input().execution_price.clone(),
                        venue_ref: fill.input().venue_ref.clone(),
                        fill_semantic_aliases: aliases,
                    },
                );
            }
            Some(existing) => existing.absorb_fill(fill)?,
        }

        match semantic_aliases.get(&fill.semantic_commitment()) {
            None => {
                semantic_aliases.insert(fill.semantic_commitment(), execution_ref);
            }
            Some(existing) if existing == &execution_ref => {}
            Some(_) => return Err(ExecutionFrontierError::FillSemanticAliasConflict),
        }
    }

    Ok((executions, semantic_aliases))
}

fn resolve_adjustments(
    executions: BTreeMap<MarketExternalIdV1, ExecutionRecord>,
    semantic_aliases: &BTreeMap<Digest32, MarketExternalIdV1>,
    adjustments: &[&CanonicalFillAdjustmentObservationV1],
) -> Result<ExecutionFrontierResolutionV1, ExecutionFrontierError> {
    let mut outgoing: BTreeMap<MarketExternalIdV1, AdjustmentEdge> = BTreeMap::new();
    let mut incoming: BTreeMap<MarketExternalIdV1, MarketExternalIdV1> = BTreeMap::new();
    let mut unresolved = false;

    for adjustment in adjustments {
        match &adjustment.input().adjustment_kind {
            FillAdjustmentKindV1::Correction {
                prior_fill_commitment,
                replacement_fill_commitment,
            } => {
                let Some(prior_execution) = semantic_aliases.get(prior_fill_commitment) else {
                    unresolved = true;
                    continue;
                };
                let Some(replacement_execution) = semantic_aliases.get(replacement_fill_commitment)
                else {
                    unresolved = true;
                    continue;
                };

                if prior_execution == replacement_execution {
                    return Err(ExecutionFrontierError::AdjustmentTopologyConflict);
                }

                match outgoing.get(prior_execution) {
                    None => {
                        outgoing.insert(
                            prior_execution.clone(),
                            AdjustmentEdge::Correction(replacement_execution.clone()),
                        );
                    }
                    Some(AdjustmentEdge::Correction(existing))
                        if existing == replacement_execution => {}
                    Some(_) => return Err(ExecutionFrontierError::AdjustmentTopologyConflict),
                }

                match incoming.get(replacement_execution) {
                    None => {
                        incoming.insert(replacement_execution.clone(), prior_execution.clone());
                    }
                    Some(existing) if existing == prior_execution => {}
                    Some(_) => return Err(ExecutionFrontierError::AdjustmentTopologyConflict),
                }
            }
            FillAdjustmentKindV1::Bust {
                prior_fill_commitment,
            } => {
                let Some(prior_execution) = semantic_aliases.get(prior_fill_commitment) else {
                    unresolved = true;
                    continue;
                };

                match outgoing.get(prior_execution) {
                    None => {
                        outgoing.insert(prior_execution.clone(), AdjustmentEdge::Bust);
                    }
                    Some(AdjustmentEdge::Bust) => {}
                    Some(_) => return Err(ExecutionFrontierError::AdjustmentTopologyConflict),
                }
            }
        }
    }

    if has_adjustment_cycle(&outgoing) {
        return Err(ExecutionFrontierError::AdjustmentCycle);
    }

    if unresolved {
        return Ok(ExecutionFrontierResolutionV1::AdjustmentResolutionIndeterminate);
    }

    let had_adjustments = !adjustments.is_empty();
    let mut effective_executions = Vec::new();
    for (execution_ref, record) in executions {
        if !outgoing.contains_key(&execution_ref) {
            effective_executions.push(record.into_effective()?);
        }
    }

    Ok(ExecutionFrontierResolutionV1::Resolved {
        had_adjustments,
        effective_executions,
    })
}

fn has_adjustment_cycle(outgoing: &BTreeMap<MarketExternalIdV1, AdjustmentEdge>) -> bool {
    for start in outgoing.keys() {
        let mut seen = BTreeSet::new();
        let mut current = start.clone();

        loop {
            if !seen.insert(current.clone()) {
                return true;
            }
            match outgoing.get(&current) {
                Some(AdjustmentEdge::Correction(next)) => current = next.clone(),
                Some(AdjustmentEdge::Bust) | None => break,
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_finance_exact::{AssetAmount, AssetId};
    use mycelix_finance_market_core::{
        canonicalize_order_intent_v1, MarketIdempotencyRefV1, MarketInstrumentRefV1,
        MarketOrderIntentInputV1, MarketProtocolIdV1, MarketSideV1, MarketSubjectRefV1,
        OrderTermsV1, QuantitySpecV1, TimeInForceV1,
    };
    use mycelix_finance_market_observation::{
        canonicalize_fill_adjustment_v1, canonicalize_fill_observation_v1,
        FillAdjustmentObservationInputV1, FillObservationInputV1,
        MarketOrderObservationSubjectV1, ProviderChronologyV1, ProviderObservationRefV1,
    };

    fn digest(byte: u8) -> Digest32 {
        Digest32::from_bytes([byte; 32])
    }

    fn profile(id: &str, byte: u8) -> MarketProfileRefV1 {
        MarketProfileRefV1 {
            profile_id: MarketProtocolIdV1::new(id).unwrap(),
            revision: 1,
            digest: digest(byte),
        }
    }

    fn subject(profile_id: &str, id: &str, byte: u8) -> MarketSubjectRefV1 {
        MarketSubjectRefV1 {
            subject_profile: profile(profile_id, byte),
            subject_id: MarketExternalIdV1::new(id).unwrap(),
        }
    }

    fn instrument() -> MarketInstrumentRefV1 {
        MarketInstrumentRefV1 {
            instrument_profile: profile("instrument.listed-equity.us", 0x22),
            instrument_id: MarketExternalIdV1::new("instrument:demo:AAPL").unwrap(),
        }
    }

    fn unit_quantity(value: u64) -> MarketQuantityV1 {
        MarketQuantityV1 {
            unit_profile: profile("quantity.micro-share-demo", 0x33),
            amount: AssetAmount::new(value, AssetId::new("AAPL.share.micro.demo").unwrap()),
        }
    }

    fn price(value: u64) -> MarketPriceV1 {
        MarketPriceV1 {
            pricing_profile: profile("price.usd-cents-per-share-demo", 0x44),
            quote_amount: AssetAmount::new(value, AssetId::new("USD.cent.demo").unwrap()),
        }
    }

    fn intent() -> CanonicalMarketOrderIntentV1 {
        canonicalize_order_intent_v1(MarketOrderIntentInputV1 {
            intent_subject: subject("mycelix.market.intent", "intent:demo:001", 0x10),
            account_subject: subject("mycelix.finance.account", "account:demo:001", 0x11),
            instrument: instrument(),
            side: MarketSideV1::AcquireLong,
            quantity: QuantitySpecV1::Units(unit_quantity(100)),
            order_terms: OrderTermsV1::Market,
            time_in_force: TimeInForceV1::Day,
            execution_profile: profile("execution.market-order-demo", 0x55),
            semantic_idempotency: MarketIdempotencyRefV1 {
                idempotency_profile: profile("idempotency.market-order-v1", 0x66),
                semantic_id: MarketProtocolIdV1::new("idem:demo:001").unwrap(),
            },
            upstream_economic_effect_commitment: None,
        })
        .unwrap()
    }

    fn scope() -> ExecutionFrontierScopeV1 {
        ExecutionFrontierScopeV1 {
            provider_profile: profile("provider.demo", 0x70),
            fill_observation_profile: profile("provider.demo.fill-observation-v1", 0x72),
            adjustment_observation_profile: profile("provider.demo.adjustment-observation-v1", 0x73),
        }
    }

    fn observation_subject(
        intent: &CanonicalMarketOrderIntentV1,
    ) -> MarketOrderObservationSubjectV1 {
        MarketOrderObservationSubjectV1 {
            intent_commitment: intent.commitment(),
            account_subject: intent.input().account_subject.clone(),
            instrument: intent.input().instrument.clone(),
            provider_profile: scope().provider_profile,
            provider_order_ref: Some(MarketExternalIdV1::new("provider-order:001").unwrap()),
        }
    }

    fn chronology(sequence: u64) -> ProviderChronologyV1 {
        ProviderChronologyV1 {
            chronology_profile: Some(profile("provider.demo.sequence-v1", 0x74)),
            sequence: Some(sequence),
            provider_time: None,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn fill(
        intent: &CanonicalMarketOrderIntentV1,
        observation_id: &str,
        execution_ref: &str,
        quantity: u64,
        price_value: u64,
        venue: Option<&str>,
        sequence: u64,
        evidence_byte: u8,
    ) -> CanonicalFillObservationV1 {
        canonicalize_fill_observation_v1(FillObservationInputV1 {
            subject: observation_subject(intent),
            observation_ref: ProviderObservationRefV1 {
                observation_profile: scope().fill_observation_profile,
                observation_id: MarketExternalIdV1::new(observation_id).unwrap(),
            },
            provider_execution_ref: MarketExternalIdV1::new(execution_ref).unwrap(),
            executed_quantity: unit_quantity(quantity),
            execution_price: price(price_value),
            venue_ref: venue.map(|value| MarketExternalIdV1::new(value).unwrap()),
            chronology: chronology(sequence),
            source_evidence_commitment: digest(evidence_byte),
        })
        .unwrap()
    }

    fn adjustment(
        intent: &CanonicalMarketOrderIntentV1,
        observation_id: &str,
        kind: FillAdjustmentKindV1,
        sequence: u64,
    ) -> CanonicalFillAdjustmentObservationV1 {
        canonicalize_fill_adjustment_v1(FillAdjustmentObservationInputV1 {
            subject: observation_subject(intent),
            observation_ref: ProviderObservationRefV1 {
                observation_profile: scope().adjustment_observation_profile,
                observation_id: MarketExternalIdV1::new(observation_id).unwrap(),
            },
            adjustment_kind: kind,
            chronology: chronology(sequence),
            source_evidence_commitment: digest(0xa0),
        })
        .unwrap()
    }

    fn effective(frontier: &ObservedExecutionFrontierV1) -> &[EffectiveObservedExecutionV1] {
        match &frontier.resolution {
            ExecutionFrontierResolutionV1::Resolved {
                effective_executions,
                ..
            } => effective_executions,
            ExecutionFrontierResolutionV1::AdjustmentResolutionIndeterminate => {
                panic!("expected resolved execution frontier")
            }
        }
    }

    #[test]
    fn positive_frontier_binds_exact_intent_and_scope() {
        let intent = intent();
        let expected_scope = scope();
        let first = fill(
            &intent,
            "fill-envelope:1",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            10,
            0x91,
        );

        let frontier = resolve_observed_execution_frontier_v1(
            &intent,
            &expected_scope,
            &[first],
            &[],
        )
        .unwrap();

        assert_eq!(frontier.intent_commitment, intent.commitment());
        assert_eq!(frontier.scope, expected_scope);
    }

    #[test]
    fn same_execution_across_two_observation_ids_counts_once() {
        let intent = intent();
        let first = fill(
            &intent,
            "fill-envelope:1",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            10,
            0x91,
        );
        let second = fill(
            &intent,
            "fill-envelope:2",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            11,
            0x92,
        );

        let frontier = resolve_observed_execution_frontier_v1(
            &intent,
            &scope(),
            &[first, second],
            &[],
        )
        .unwrap();

        assert_eq!(frontier.unique_fill_observation_count, 2);
        assert_eq!(frontier.unique_execution_count, 1);
        assert_eq!(effective(&frontier).len(), 1);
        assert_eq!(effective(&frontier)[0].supporting_observation_count, 2);
    }

    #[test]
    fn venue_can_be_monotonically_enriched() {
        let intent = intent();
        let first = fill(
            &intent,
            "fill-envelope:1",
            "exec:1",
            40,
            18_750,
            None,
            10,
            0x91,
        );
        let enriched = fill(
            &intent,
            "fill-envelope:2",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            11,
            0x92,
        );

        let frontier = resolve_observed_execution_frontier_v1(
            &intent,
            &scope(),
            &[first, enriched],
            &[],
        )
        .unwrap();

        assert_eq!(effective(&frontier).len(), 1);
        assert_eq!(
            effective(&frontier)[0].venue_ref.as_ref().map(MarketExternalIdV1::as_str),
            Some("venue:demo")
        );
    }

    #[test]
    fn incompatible_non_empty_venues_conflict() {
        let intent = intent();
        let first = fill(
            &intent,
            "fill-envelope:1",
            "exec:1",
            40,
            18_750,
            Some("venue:a"),
            10,
            0x91,
        );
        let changed = fill(
            &intent,
            "fill-envelope:2",
            "exec:1",
            40,
            18_750,
            Some("venue:b"),
            11,
            0x92,
        );

        assert_eq!(
            resolve_observed_execution_frontier_v1(
                &intent,
                &scope(),
                &[first, changed],
                &[],
            ),
            Err(ExecutionFrontierError::ExecutionIdentityConflict)
        );
    }

    #[test]
    fn same_execution_ref_with_changed_quantity_conflicts() {
        let intent = intent();
        let first = fill(
            &intent,
            "fill-envelope:1",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            10,
            0x91,
        );
        let changed = fill(
            &intent,
            "fill-envelope:2",
            "exec:1",
            41,
            18_750,
            Some("venue:demo"),
            11,
            0x92,
        );

        assert_eq!(
            resolve_observed_execution_frontier_v1(
                &intent,
                &scope(),
                &[first, changed],
                &[],
            ),
            Err(ExecutionFrontierError::ExecutionIdentityConflict)
        );
    }

    #[test]
    fn identical_economics_with_different_execution_refs_are_distinct() {
        let intent = intent();
        let first = fill(
            &intent,
            "fill-envelope:1",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            10,
            0x91,
        );
        let second = fill(
            &intent,
            "fill-envelope:2",
            "exec:2",
            40,
            18_750,
            Some("venue:demo"),
            11,
            0x92,
        );

        let frontier = resolve_observed_execution_frontier_v1(
            &intent,
            &scope(),
            &[first, second],
            &[],
        )
        .unwrap();

        assert_eq!(frontier.unique_execution_count, 2);
        assert_eq!(effective(&frontier).len(), 2);
    }

    #[test]
    fn correction_can_reference_one_alias_of_duplicated_execution() {
        let intent = intent();
        let prior_first = fill(
            &intent,
            "fill-envelope:1",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            10,
            0x91,
        );
        let prior_alias = fill(
            &intent,
            "fill-envelope:2",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            11,
            0x92,
        );
        let replacement = fill(
            &intent,
            "fill-envelope:3",
            "exec:2",
            45,
            18_700,
            Some("venue:demo"),
            12,
            0x93,
        );
        let correction = adjustment(
            &intent,
            "adjust:1",
            FillAdjustmentKindV1::Correction {
                prior_fill_commitment: prior_alias.semantic_commitment(),
                replacement_fill_commitment: replacement.semantic_commitment(),
            },
            20,
        );

        let frontier = resolve_observed_execution_frontier_v1(
            &intent,
            &scope(),
            &[prior_first, prior_alias, replacement],
            &[correction],
        )
        .unwrap();

        assert_eq!(frontier.unique_execution_count, 2);
        assert_eq!(effective(&frontier).len(), 1);
        assert_eq!(
            effective(&frontier)[0].provider_execution_ref.as_str(),
            "exec:2"
        );
    }

    #[test]
    fn correction_between_two_aliases_of_same_execution_conflicts() {
        let intent = intent();
        let first = fill(
            &intent,
            "fill-envelope:1",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            10,
            0x91,
        );
        let alias = fill(
            &intent,
            "fill-envelope:2",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            11,
            0x92,
        );
        let correction = adjustment(
            &intent,
            "adjust:1",
            FillAdjustmentKindV1::Correction {
                prior_fill_commitment: first.semantic_commitment(),
                replacement_fill_commitment: alias.semantic_commitment(),
            },
            20,
        );

        assert_eq!(
            resolve_observed_execution_frontier_v1(
                &intent,
                &scope(),
                &[first, alias],
                &[correction],
            ),
            Err(ExecutionFrontierError::AdjustmentTopologyConflict)
        );
    }

    #[test]
    fn missing_adjustment_reference_is_indeterminate() {
        let intent = intent();
        let first = fill(
            &intent,
            "fill-envelope:1",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            10,
            0x91,
        );
        let bust = adjustment(
            &intent,
            "adjust:1",
            FillAdjustmentKindV1::Bust {
                prior_fill_commitment: digest(0xff),
            },
            20,
        );

        let frontier = resolve_observed_execution_frontier_v1(
            &intent,
            &scope(),
            &[first],
            &[bust],
        )
        .unwrap();

        assert_eq!(
            frontier.resolution,
            ExecutionFrontierResolutionV1::AdjustmentResolutionIndeterminate
        );
    }

    #[test]
    fn bust_removes_execution_from_effective_frontier() {
        let intent = intent();
        let first = fill(
            &intent,
            "fill-envelope:1",
            "exec:1",
            40,
            18_750,
            Some("venue:demo"),
            10,
            0x91,
        );
        let bust = adjustment(
            &intent,
            "adjust:1",
            FillAdjustmentKindV1::Bust {
                prior_fill_commitment: first.semantic_commitment(),
            },
            20,
        );

        let frontier = resolve_observed_execution_frontier_v1(
            &intent,
            &scope(),
            &[first],
            &[bust],
        )
        .unwrap();

        assert_eq!(frontier.unique_execution_count, 1);
        assert!(effective(&frontier).is_empty());
    }
}
