//! Deterministic derived normalization for financial observations.
//!
//! Raw MarketObservation records are immutable inputs. This module creates
//! independently identifiable projections for unit conversion, FX conversion,
//! corporate-action adjustment, precision normalization, and session semantics.
//!
//! No provider, network, execution, authorization, or floating-point dependency
//! is introduced. Arithmetic is performed on decimal text using bounded i128
//! intermediates so replay does not depend on machine floating-point behavior.

use super::{DecimalValue, EvidenceRef, EvidenceStatus, InformationFrontier, MarketObservation, MarketUnit};
use serde::{Deserialize, Serialize};

/// A deterministic recipe describing how a raw observation becomes a derived
/// projection. The recipe itself carries no authority to mutate source data.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NormalizationRecipe {
    pub recipe_id: String,
    pub adjustment: AdjustmentKind,
    pub target_unit: Option<MarketUnit>,
    pub multiplier: Option<DecimalValue>,
    pub output_scale: Option<u32>,
    pub session: Option<SessionNormalization>,
    /// Evidence required to perform the transformation. Every reference must
    /// be available at the replay frontier; otherwise the transformation fails closed.
    pub evidence_refs: Vec<EvidenceRef>,
}

/// Explicitly typed transformations prevent provider-specific adjustment
/// semantics from becoming implicit global truth.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AdjustmentKind {
    None,
    Split,
    Dividend,
    CurrencyConversion,
    UnitConversion,
    SessionNormalization,
    Other(String),
}

/// Session handling is a projection concern, not a rewrite of observation time.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SessionNormalization {
    pub venue_id: String,
    pub session_label: String,
    pub timezone: String,
}

/// Reference to the immutable raw observation(s) used by a projection.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NormalizationInputRef {
    RawObservation { observation_id: String, source_id: String },
    DerivedProjection { projection_id: String, transformation_id: String },
    Evidence { evidence_id: String, source_id: String },
}

/// Independently identifiable normalized projection with complete recipe lineage.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NormalizedObservation {
    pub projection_id: String,
    pub input_refs: Vec<NormalizationInputRef>,
    pub recipe: NormalizationRecipe,
    pub value: DecimalValue,
    pub unit: MarketUnit,
    pub observed_at_micros: i64,
    pub information_frontier: super::InformationFrontier,
}


/// A validated ordered chain of normalization recipes.
///
/// Each step consumes the exact output of the previous step. The chain is
/// itself immutable metadata, so replay can verify both ordering and recipe
/// identity rather than trusting a precomputed final number.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NormalizationChain {
    pub chain_id: String,
    pub steps: Vec<NormalizationRecipe>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ChainError {
    EmptyChain,
    DuplicateRecipeId,
    Step(NormalizationError),
}

/// Apply a complete ordered chain while preserving every intermediate result.
pub fn normalize_chain(
    observation: &MarketObservation,
    chain: &NormalizationChain,
) -> Result<Vec<NormalizedObservation>, ChainError> {
    if chain.steps.is_empty() {
        return Err(ChainError::EmptyChain);
    }
    let mut seen = std::collections::BTreeSet::new();
    let mut current = observation.clone();
    let mut results = Vec::with_capacity(chain.steps.len());
    for recipe in &chain.steps {
        if !seen.insert(recipe.recipe_id.clone()) {
            return Err(ChainError::DuplicateRecipeId);
        }
        let prior_projection_id = current.observation_id.clone();
        let mut next = normalize_observation(&current, recipe).map_err(ChainError::Step)?;
        if !results.is_empty() {
            next.input_refs = vec![NormalizationInputRef::DerivedProjection {
                projection_id: prior_projection_id,
                transformation_id: results.last().unwrap().recipe.recipe_id.clone(),
            }];
        }
        let mut lineage = current.evidence_refs.clone();
        lineage.extend(recipe.evidence_refs.clone());
        lineage.sort_by(|a, b| a.evidence_id.cmp(&b.evidence_id));
        lineage.dedup_by(|a, b| a.evidence_id == b.evidence_id);
        next.evidence_refs = lineage.clone();
        current.value = next.value.clone();
        current.unit = next.unit.clone();
        current.observation_id = next.projection_id.clone();
        current.evidence_refs = lineage;
        results.push(next);
    }
    Ok(results)
}

/// Errors are explicit so a failed normalization cannot silently produce a value.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NormalizationError {
    MissingMultiplier,
    InvalidDecimal,
    ScaleOverflow,
    ArithmeticOverflow,
    NegativeScale,
    EvidenceUnavailableAtFrontier,
    ProtectedEvidence,
    ConflictingEvidence,
    FutureObservation,
    FrontierMismatch,
}

/// Apply a single deterministic multiplier and optional output scale.
///
/// Decimal strings may contain a leading sign and a decimal point. The scale
/// field is authoritative and must agree with the number of fractional digits.
pub fn normalize_observation(
    observation: &MarketObservation,
    recipe: &NormalizationRecipe,
) -> Result<NormalizedObservation, NormalizationError> {
    normalize_observation_at_frontier(observation, recipe, &observation.information_frontier)
}

/// Apply a transformation only with information available at the requested frontier.
pub fn normalize_observation_at_frontier(
    observation: &MarketObservation,
    recipe: &NormalizationRecipe,
    frontier: &InformationFrontier,
) -> Result<NormalizedObservation, NormalizationError> {
    validate_frontier(observation, recipe, frontier)?;
    let output_unit = recipe
        .target_unit
        .clone()
        .unwrap_or_else(|| observation.unit.clone());

    let value = match recipe.multiplier.as_ref() {
        Some(multiplier) => multiply_decimal(&observation.value, multiplier)?,
        None => observation.value.clone(),
    };

    let value = match recipe.output_scale {
        Some(scale) => rescale_decimal(&value, scale)?,
        None => value,
    };

    let projection_id = format!(
        "norm:{}:{}:{}",
        observation.observation_id, recipe.recipe_id, observation.information_frontier.frontier_id
    );

    Ok(NormalizedObservation {
        projection_id,
        input_refs: vec![NormalizationInputRef::RawObservation {
            observation_id: observation.observation_id.clone(),
            source_id: observation.source.source_id.clone(),
        }],
        evidence_refs: recipe.evidence_refs.clone(),
        recipe: recipe.clone(),
        value,
        unit: output_unit,
        observed_at_micros: observation.observed_at_micros,
        information_frontier: observation.information_frontier.clone(),
    })
}

fn validate_frontier(
    observation: &MarketObservation,
    recipe: &NormalizationRecipe,
    frontier: &InformationFrontier,
) -> Result<(), NormalizationError> {
    if observation.information_frontier.as_of_micros > frontier.as_of_micros {
        return Err(NormalizationError::FrontierMismatch);
    }
    if observation.status == EvidenceStatus::Protected {
        return Err(NormalizationError::ProtectedEvidence);
    }
    if observation.status == EvidenceStatus::Conflicting {
        return Err(NormalizationError::ConflictingEvidence);
    }
    if observation.available_at_micros.is_none() {
        return Err(NormalizationError::EvidenceUnavailableAtFrontier);
    }
    if observation.available_at_micros.unwrap() > frontier.as_of_micros {
        return Err(NormalizationError::FutureObservation);
    }
    for evidence in &recipe.evidence_refs {
        match evidence.status {
            EvidenceStatus::Protected => return Err(NormalizationError::ProtectedEvidence),
            EvidenceStatus::Conflicting => return Err(NormalizationError::ConflictingEvidence),
            EvidenceStatus::Unavailable | EvidenceStatus::Unknown | EvidenceStatus::FutureInaccessible | EvidenceStatus::Stale => {
                return Err(NormalizationError::EvidenceUnavailableAtFrontier)
            }
            EvidenceStatus::Known | EvidenceStatus::ObservedUnqualified => {}
        }
        if evidence.available_at_micros.is_none() || evidence.available_at_micros.unwrap() > frontier.as_of_micros {
            return Err(NormalizationError::EvidenceUnavailableAtFrontier);
        }
        if evidence.information_frontier.as_of_micros > frontier.as_of_micros {
            return Err(NormalizationError::FrontierMismatch);
        }
    }
    Ok(())
}

fn parse_decimal(value: &DecimalValue) -> Result<i128, NormalizationError> {
    if value.value.trim() != value.value || value.value.is_empty() {
        return Err(NormalizationError::InvalidDecimal);
    }
    let (negative, digits) = match value.value.as_bytes().first() {
        Some(b'-') => (true, &value.value[1..]),
        Some(b'+') => (false, &value.value[1..]),
        _ => (false, value.value.as_str()),
    };
    if digits.is_empty() {
        return Err(NormalizationError::InvalidDecimal);
    }
    let mut dot_seen = false;
    let mut fractional = 0u32;
    let mut integer = String::new();
    for byte in digits.bytes() {
        match byte {
            b'0'..=b'9' => {
                integer.push(byte as char);
                if dot_seen {
                    fractional = fractional.checked_add(1).ok_or(NormalizationError::ScaleOverflow)?;
                }
            }
            b'.' if !dot_seen => dot_seen = true,
            _ => return Err(NormalizationError::InvalidDecimal),
        }
    }
    if fractional != value.scale {
        return Err(NormalizationError::InvalidDecimal);
    }
    let parsed = integer.parse::<i128>().map_err(|_| NormalizationError::ArithmeticOverflow)?;
    Ok(if negative { -parsed } else { parsed })
}

fn decimal_from_scaled(mut integer: i128, scale: u32) -> DecimalValue {
    let negative = integer < 0;
    if negative {
        integer = -integer;
    }
    let mut digits = integer.to_string();
    if scale > 0 {
        let scale_usize = scale as usize;
        if digits.len() <= scale_usize {
            digits = format!("{}{}", "0".repeat(scale_usize + 1 - digits.len()), digits);
        }
        let split = digits.len() - scale_usize;
        digits.insert(split, '.');
    }
    if negative && integer != 0 {
        digits.insert(0, '-');
    }
    DecimalValue { value: digits, scale }
}

fn multiply_decimal(left: &DecimalValue, right: &DecimalValue) -> Result<DecimalValue, NormalizationError> {
    let l = parse_decimal(left)?;
    let r = parse_decimal(right)?;
    let product = l.checked_mul(r).ok_or(NormalizationError::ArithmeticOverflow)?;
    let scale = left.scale.checked_add(right.scale).ok_or(NormalizationError::ScaleOverflow)?;
    Ok(decimal_from_scaled(product, scale))
}

/// Rescale exactly when reducing precision would discard non-zero digits;
/// callers must opt into a larger scale or an exact lower-scale representation.
fn rescale_decimal(value: &DecimalValue, target_scale: u32) -> Result<DecimalValue, NormalizationError> {
    let integer = parse_decimal(value)?;
    if target_scale == value.scale {
        return Ok(value.clone());
    }
    if target_scale > value.scale {
        let factor = pow10(target_scale - value.scale)?;
        let scaled = integer.checked_mul(factor).ok_or(NormalizationError::ArithmeticOverflow)?;
        return Ok(decimal_from_scaled(scaled, target_scale));
    }
    let factor = pow10(value.scale - target_scale)?;
    if integer % factor != 0 {
        return Err(NormalizationError::ScaleOverflow);
    }
    Ok(decimal_from_scaled(integer / factor, target_scale))
}

fn pow10(scale: u32) -> Result<i128, NormalizationError> {
    let mut result = 1i128;
    for _ in 0..scale {
        result = result.checked_mul(10).ok_or(NormalizationError::ArithmeticOverflow)?;
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        EvidenceStatus, FinancialSubjectKind, FinancialSubjectRef, IdentifierAlias,
        InformationFrontier, SourceRef, ValidityInterval,
    };

    fn observation(value: &str, scale: u32) -> MarketObservation {
        MarketObservation {
            observation_id: "obs-raw-001".into(),
            subject: FinancialSubjectRef {
                subject_id: "instrument:example".into(),
                subject_kind: FinancialSubjectKind::Instrument,
                validity_interval: ValidityInterval { valid_from_micros: Some(0), valid_to_micros: None },
                identifier_aliases: vec![IdentifierAlias {
                    namespace: "ticker".into(), value: "EXM".into(),
                    valid_from_micros: Some(0), valid_to_micros: None,
                }],
                lineage_relations: vec![],
                information_frontier: InformationFrontier {
                    as_of_micros: 1_700_000_000_000_000,
                    frontier_id: "frontier-1".into(),
                },
            },
            observed_at_micros: 1_700_000_000_000_100,
            published_at_micros: Some(1_700_000_000_000_200),
            available_at_micros: Some(1_700_000_000_000_300),
            ingested_at_micros: 1_700_000_000_000_400,
            unit: MarketUnit::Price { currency: "USD".into() },
            value: DecimalValue { value: value.into(), scale },
            status: EvidenceStatus::ObservedUnqualified,
            source: SourceRef {
                source_id: "provider-observation-1".into(),
                provider_id: "provider-a".into(),
                common_ancestry_id: Some("upstream-1".into()),
                retrieved_at_micros: 1_700_000_000_000_400,
            },
            evidence_refs: vec![],
            information_frontier: InformationFrontier {
                as_of_micros: 1_700_000_000_000_000,
                frontier_id: "frontier-1".into(),
            },
        }
    }

    fn recipe(adjustment: AdjustmentKind, multiplier: Option<&str>, multiplier_scale: u32, output_scale: Option<u32>) -> NormalizationRecipe {
        NormalizationRecipe {
            recipe_id: "recipe-1".into(),
            adjustment,
            target_unit: Some(MarketUnit::Price { currency: "EUR".into() }),
            multiplier: multiplier.map(|v| DecimalValue { value: v.into(), scale: multiplier_scale }),
            output_scale,
            session: None,
            evidence_refs: vec![],
    }

    #[test]
    fn split_adjustment_is_derived_and_does_not_mutate_raw() {
        let raw = observation("100.00", 2);
        let normalized = normalize_observation(&raw, &recipe(AdjustmentKind::Split, Some("0.5"), 1, Some(2))).unwrap();
        assert_eq!(normalized.value.value, "50.00");
        assert_eq!(raw.value.value, "100.00");
        assert_eq!(match &normalized.input_refs[0] { NormalizationInputRef::RawObservation { observation_id, .. } => observation_id.as_str(), _ => "" }, "obs-raw-001");
    }

    #[test]
    fn dividend_adjustment_has_explicit_recipe() {
        let raw = observation("101.00", 2);
        let normalized = normalize_observation(&raw, &recipe(AdjustmentKind::Dividend, Some("0.99"), 2, Some(2))).unwrap();
        assert_eq!(normalized.value.value, "99.99");
        assert_eq!(normalized.recipe.adjustment, AdjustmentKind::Dividend);
    }

    #[test]
    fn fx_conversion_is_exact_decimal_not_float() {
        let raw = observation("123.45", 2);
        let normalized = normalize_observation(&raw, &recipe(AdjustmentKind::CurrencyConversion, Some("0.91"), 2, Some(4))).unwrap();
        assert_eq!(normalized.value.value, "112.3395");
        assert_eq!(normalized.unit, MarketUnit::Price { currency: "EUR".into() });
    }

    #[test]
    fn precision_increase_is_replayable() {
        let raw = observation("12.3", 1);
        let normalized = normalize_observation(&raw, &recipe(AdjustmentKind::UnitConversion, None, 0, Some(3))).unwrap();
        assert_eq!(normalized.value.value, "12.300");
    }

    #[test]
    fn lossy_precision_reduction_is_rejected() {
        let raw = observation("12.345", 3);
        let result = normalize_observation(&raw, &recipe(AdjustmentKind::UnitConversion, None, 0, Some(2)));
        assert_eq!(result, Err(NormalizationError::ScaleOverflow));
    }

    #[test]
    fn timezone_session_is_projection_metadata() {
        let raw = observation("10.00", 2);
        let normalized = normalize_observation(&raw, &NormalizationRecipe {
            recipe_id: "session-1".into(),
            adjustment: AdjustmentKind::SessionNormalization,
            target_unit: None,
            multiplier: None,
            output_scale: None,
            session: Some(SessionNormalization {
                venue_id: "venue-x".into(),
                session_label: "regular".into(),
                timezone: "UTC".into(),
            }),
            evidence_refs: vec![],
        }).unwrap();
        assert_eq!(normalized.observed_at_micros, raw.observed_at_micros);
        assert_eq!(normalized.recipe.session.as_ref().unwrap().timezone, "UTC");
    }

    #[test]
    fn invalid_multiplier_fails_closed() {
        let raw = observation("10.00", 2);
        let result = normalize_observation(&raw, &recipe(AdjustmentKind::CurrencyConversion, Some("bad"), 0, None));
        assert_eq!(result, Err(NormalizationError::InvalidDecimal));
    }

    #[test]
    fn chain_preserves_derived_projection_lineage() {
        let raw = observation("100.00", 2);
        let first = recipe(AdjustmentKind::Split, Some("0.5"), 1, Some(2));
        let second = NormalizationRecipe { recipe_id: "recipe-2".into(), ..recipe(AdjustmentKind::CurrencyConversion, Some("0.9"), 1, Some(2)) };
        let chain = NormalizationChain { chain_id: "chain-1".into(), steps: vec![first, second] };
        let results = normalize_chain(&raw, &chain).unwrap();
        assert!(matches!(results[1].input_refs[0], NormalizationInputRef::DerivedProjection { .. }));
        assert_eq!(results[1].evidence_refs.len(), 0);
    }

    #[test]
    fn projection_identity_includes_recipe_and_frontier() {
        let raw = observation("10.00", 2);
        let normalized = normalize_observation(&raw, &recipe(AdjustmentKind::Split, Some("0.5"), 1, Some(2))).unwrap();
        assert_eq!(normalized.projection_id, "norm:obs-raw-001:recipe-1:frontier-1");
    }
}


#[cfg(test)]
mod frontier_tests {
    use super::*;
    use crate::{FinancialSubjectKind, FinancialSubjectRef, IdentifierAlias, SourceRef, ValidityInterval};
    fn obs() -> MarketObservation {
        MarketObservation { observation_id: "obs-frontier-001".into(),
            subject: FinancialSubjectRef { subject_id: "instrument:x".into(), subject_kind: FinancialSubjectKind::Instrument,
                validity_interval: ValidityInterval { valid_from_micros: Some(0), valid_to_micros: None },
                identifier_aliases: vec![IdentifierAlias { namespace: "ticker".into(), value: "X".into(), valid_from_micros: Some(0), valid_to_micros: None }], lineage_relations: vec![],
                information_frontier: InformationFrontier { as_of_micros: 100, frontier_id: "f100".into() } },
            observed_at_micros: 90, published_at_micros: Some(95), available_at_micros: Some(96), ingested_at_micros: 97,
            unit: MarketUnit::Price { currency: "USD".into() }, value: DecimalValue { value: "100.00".into(), scale: 2 }, status: EvidenceStatus::ObservedUnqualified,
            source: SourceRef { source_id: "src-x".into(), provider_id: "p-x".into(), common_ancestry_id: None, retrieved_at_micros: 97 }, evidence_refs: vec![],
            information_frontier: InformationFrontier { as_of_micros: 100, frontier_id: "f100".into() } }
    }
    fn frontier(at: i64) -> InformationFrontier { InformationFrontier { as_of_micros: at, frontier_id: format!("f{at}") } }
    fn fx_evidence(available: i64, status: EvidenceStatus) -> EvidenceRef {
        let o=obs(); EvidenceRef { evidence_id: "fx-1".into(), artifact_id: None, source: o.source, status, observed_at_micros: Some(available-1), available_at_micros: Some(available), information_frontier: frontier(available) }
    }
    fn fx_recipe(e: EvidenceRef) -> NormalizationRecipe {
        NormalizationRecipe { recipe_id: "fx-r1".into(), adjustment: AdjustmentKind::CurrencyConversion, target_unit: Some(MarketUnit::Price { currency: "EUR".into() }), multiplier: Some(DecimalValue { value: "0.9".into(), scale: 1 }), output_scale: Some(2), session: None, evidence_refs: vec![e] }
    }
    #[test] fn future_fx_is_rejected_at_historical_frontier() { assert_eq!(normalize_observation_at_frontier(&obs(), &fx_recipe(fx_evidence(150, EvidenceStatus::Known)), &frontier(120)), Err(NormalizationError::EvidenceUnavailableAtFrontier)); }
    #[test] fn protected_fx_is_rejected_even_when_available() { assert_eq!(normalize_observation_at_frontier(&obs(), &fx_recipe(fx_evidence(105, EvidenceStatus::Protected)), &frontier(120)), Err(NormalizationError::ProtectedEvidence)); }
    #[test] fn available_fx_can_be_used_at_frontier() { let r=normalize_observation_at_frontier(&obs(), &fx_recipe(fx_evidence(105, EvidenceStatus::Known)), &frontier(120)).unwrap(); assert_eq!(r.evidence_refs.len(), 1); }
}
