// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-012 qualified supplier-attributed procurement award values.
//!
//! AC-012 defines exact monetary semantics for value-weighted procurement supplier
//! concentration. It supports one currency per calculation, exact decimal scaling,
//! and supplier-attributed award blocks only. It performs no rounding or FX.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::capture_metrics::{DerivedCaptureObservation, ObservationContext};
use crate::capture_observation::{
    CaptureContract, CaptureMetric, CaptureObservation, Measurement, MetricValue, ProvenanceRef,
    Uncertainty,
};
use crate::institutional_graph::{
    InstitutionalEdge, InstitutionalGraphContract, InstitutionalGraphViolation,
    InstitutionalRelationKind, ProcurementRole,
};

pub const MAX_MONETARY_DECIMAL_SCALE: u8 = 18;
pub const VALUE_WEIGHTED_SUPPLIER_HHI_METHOD: &str =
    "mycelix:ac-012:procurement-value-weighted-supplier-hhi:v1";

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ExactCurrencyAmount {
    /// Non-negative decimal coefficient. `coefficient / 10^scale` is the amount.
    pub coefficient: u64,
    pub scale: u8,
    /// Uppercase three-letter currency code. Registry membership is bound through
    /// `currency_registry_ref` and its provenance, not hard-coded in this crate.
    pub currency: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct SupplierAttributedAwardValue {
    pub value_ref: String,
    pub award_edge_ref: String,
    /// Upstream award-block reference. It must be unique in the value set so a
    /// joint multi-supplier award total cannot be duplicated across suppliers.
    pub award_ref: String,
    pub amount: ExactCurrencyAmount,
    pub provenance: Vec<ProvenanceRef>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProcurementAwardValueSet {
    pub snapshot_ref: String,
    pub value_semantics_ref: String,
    pub currency_registry_ref: String,
    pub currency_registry_provenance: Vec<ProvenanceRef>,
    pub records: Vec<SupplierAttributedAwardValue>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ValueWeightedProcurementObservation {
    pub derived: DerivedCaptureObservation,
    pub value_snapshot_ref: String,
    pub value_record_refs: Vec<String>,
    pub currency: String,
    pub normalized_scale: u8,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ProcurementValueError {
    MissingSnapshotReference,
    MissingValueSemanticsReference,
    MissingCurrencyRegistryReference,
    InvalidCurrencyRegistryProvenance,
    EmptyValueSet,
    InvalidValueRecord { record_index: usize },
    InvalidCurrencyCode { record_index: usize },
    DecimalScaleTooLarge { record_index: usize },
    DuplicateValueReference { value_ref: String },
    DuplicateAwardEdgeReference { award_edge_ref: String },
    DuplicateAwardReference { award_ref: String },
    MixedCurrencies,
    AwardEdgeInvalid {
        edge_index: usize,
        violations: Vec<InstitutionalGraphViolation>,
    },
    MissingAwardEdgeReference,
    DuplicateAwardEdgeId { edge_id: String },
    ValueReferencesUnknownAwardEdge { award_edge_ref: String },
    MissingValueForAwardEdge { award_edge_ref: String },
    ArithmeticOverflow,
    ZeroTotalAwardValue,
    OutputContractViolation,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ProcurementValueContract;

impl ProcurementValueContract {
    pub fn value_weighted_supplier_concentration(
        edges: &[InstitutionalEdge],
        values: &ProcurementAwardValueSet,
        mut context: ObservationContext,
    ) -> Result<ValueWeightedProcurementObservation, Vec<ProcurementValueError>> {
        validate_value_set(values)?;

        let mut award_edges = BTreeMap::<String, &InstitutionalEdge>::new();
        let mut errors = Vec::new();
        for (edge_index, edge) in edges.iter().enumerate() {
            if !matches!(
                &edge.relation,
                InstitutionalRelationKind::ProcurementParticipation {
                    role: ProcurementRole::Awardee
                }
            ) {
                continue;
            }
            if edge.id.trim().is_empty() {
                errors.push(ProcurementValueError::MissingAwardEdgeReference);
                continue;
            }
            if let Err(violations) = InstitutionalGraphContract::validate_edge(edge) {
                errors.push(ProcurementValueError::AwardEdgeInvalid {
                    edge_index,
                    violations,
                });
            }
            if award_edges.insert(edge.id.clone(), edge).is_some() {
                errors.push(ProcurementValueError::DuplicateAwardEdgeId {
                    edge_id: edge.id.clone(),
                });
            }
        }
        if award_edges.is_empty() {
            errors.push(ProcurementValueError::MissingAwardEdgeReference);
        }

        let record_by_edge: BTreeMap<_, _> = values
            .records
            .iter()
            .map(|record| (record.award_edge_ref.as_str(), record))
            .collect();
        for record in &values.records {
            if !award_edges.contains_key(&record.award_edge_ref) {
                errors.push(ProcurementValueError::ValueReferencesUnknownAwardEdge {
                    award_edge_ref: record.award_edge_ref.clone(),
                });
            }
        }
        for edge_ref in award_edges.keys() {
            if !record_by_edge.contains_key(edge_ref.as_str()) {
                errors.push(ProcurementValueError::MissingValueForAwardEdge {
                    award_edge_ref: edge_ref.clone(),
                });
            }
        }
        if !errors.is_empty() {
            return Err(errors);
        }

        let normalized_scale = values
            .records
            .iter()
            .map(|record| record.amount.scale)
            .max()
            .unwrap_or(0);
        let currency = values.records[0].amount.currency.clone();
        let mut supplier_weights = BTreeMap::<String, u128>::new();
        for record in &values.records {
            let edge = award_edges
                .get(&record.award_edge_ref)
                .expect("validated award-edge reference");
            let normalized = normalize_amount(&record.amount, normalized_scale)?;
            let entry = supplier_weights.entry(edge.from.id.clone()).or_insert(0);
            *entry = entry
                .checked_add(normalized)
                .ok_or_else(|| vec![ProcurementValueError::ArithmeticOverflow])?;
        }

        let total = checked_sum(supplier_weights.values().copied())?;
        if total == 0 {
            return Err(vec![ProcurementValueError::ZeroTotalAwardValue]);
        }
        let numerator = checked_sum_squares(supplier_weights.values().copied())?;
        let denominator = total
            .checked_mul(total)
            .ok_or_else(|| vec![ProcurementValueError::ArithmeticOverflow])?;

        let metric_value = MetricValue {
            numerator: i64::try_from(numerator)
                .map_err(|_| vec![ProcurementValueError::ArithmeticOverflow])?,
            denominator: u64::try_from(denominator)
                .map_err(|_| vec![ProcurementValueError::ArithmeticOverflow])?,
            unit: "hhi_ratio".into(),
        };

        context.limitations.push(
            "supplier concentration is weighted by exact supplier-attributed award values; mixed currencies, FX conversion, negative/concession values, joint supplier totals, contract values, and implementation payments are outside AC-012 v1"
                .into(),
        );

        let mut provenance_set = BTreeSet::new();
        for edge in award_edges.values() {
            for provenance in &edge.provenance {
                provenance_set.insert((
                    provenance.source_ref.clone(),
                    provenance.content_hash.clone(),
                ));
            }
        }
        for record in &values.records {
            for provenance in &record.provenance {
                provenance_set.insert((
                    provenance.source_ref.clone(),
                    provenance.content_hash.clone(),
                ));
            }
        }
        for provenance in &values.currency_registry_provenance {
            provenance_set.insert((
                provenance.source_ref.clone(),
                provenance.content_hash.clone(),
            ));
        }
        let provenance = provenance_set
            .into_iter()
            .map(|(source_ref, content_hash)| ProvenanceRef {
                source_ref,
                content_hash,
            })
            .collect();

        let observation = CaptureObservation {
            id: context.observation_id,
            subject: context.subject,
            measurement: Measurement {
                metric: CaptureMetric::ProcurementSupplierConcentration,
                value: metric_value,
                method_ref: VALUE_WEIGHTED_SUPPLIER_HHI_METHOD.into(),
            },
            provenance,
            uncertainty: Uncertainty {
                confidence: context.confidence,
                limitations: context.limitations,
            },
            observed_at: context.observed_at,
        };
        if CaptureContract::validate_observation(&observation).is_err() {
            return Err(vec![ProcurementValueError::OutputContractViolation]);
        }

        let input_edge_refs = award_edges.keys().cloned().collect();
        let mut value_record_refs: Vec<_> = values
            .records
            .iter()
            .map(|record| record.value_ref.clone())
            .collect();
        value_record_refs.sort();

        Ok(ValueWeightedProcurementObservation {
            derived: DerivedCaptureObservation {
                observation,
                input_edge_refs,
            },
            value_snapshot_ref: values.snapshot_ref.clone(),
            value_record_refs,
            currency,
            normalized_scale,
        })
    }
}

fn validate_value_set(
    values: &ProcurementAwardValueSet,
) -> Result<(), Vec<ProcurementValueError>> {
    let mut errors = Vec::new();
    if values.snapshot_ref.trim().is_empty() {
        errors.push(ProcurementValueError::MissingSnapshotReference);
    }
    if values.value_semantics_ref.trim().is_empty() {
        errors.push(ProcurementValueError::MissingValueSemanticsReference);
    }
    if values.currency_registry_ref.trim().is_empty() {
        errors.push(ProcurementValueError::MissingCurrencyRegistryReference);
    }
    if values.currency_registry_provenance.is_empty()
        || values
            .currency_registry_provenance
            .iter()
            .any(|provenance| !strict_provenance(provenance))
    {
        errors.push(ProcurementValueError::InvalidCurrencyRegistryProvenance);
    }
    if values.records.is_empty() {
        errors.push(ProcurementValueError::EmptyValueSet);
        return Err(errors);
    }

    let mut value_refs = BTreeSet::new();
    let mut edge_refs = BTreeSet::new();
    let mut award_refs = BTreeSet::new();
    let mut currencies = BTreeSet::new();
    for (record_index, record) in values.records.iter().enumerate() {
        if record.value_ref.trim().is_empty()
            || record.award_edge_ref.trim().is_empty()
            || record.award_ref.trim().is_empty()
            || record.provenance.is_empty()
            || record
                .provenance
                .iter()
                .any(|provenance| !strict_provenance(provenance))
        {
            errors.push(ProcurementValueError::InvalidValueRecord { record_index });
        }
        if !valid_currency_code(&record.amount.currency) {
            errors.push(ProcurementValueError::InvalidCurrencyCode { record_index });
        }
        if record.amount.scale > MAX_MONETARY_DECIMAL_SCALE {
            errors.push(ProcurementValueError::DecimalScaleTooLarge { record_index });
        }
        if !value_refs.insert(record.value_ref.clone()) {
            errors.push(ProcurementValueError::DuplicateValueReference {
                value_ref: record.value_ref.clone(),
            });
        }
        if !edge_refs.insert(record.award_edge_ref.clone()) {
            errors.push(ProcurementValueError::DuplicateAwardEdgeReference {
                award_edge_ref: record.award_edge_ref.clone(),
            });
        }
        if !award_refs.insert(record.award_ref.clone()) {
            errors.push(ProcurementValueError::DuplicateAwardReference {
                award_ref: record.award_ref.clone(),
            });
        }
        currencies.insert(record.amount.currency.clone());
    }
    if currencies.len() > 1 {
        errors.push(ProcurementValueError::MixedCurrencies);
    }

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

fn normalize_amount(
    amount: &ExactCurrencyAmount,
    target_scale: u8,
) -> Result<u128, Vec<ProcurementValueError>> {
    let exponent = target_scale
        .checked_sub(amount.scale)
        .ok_or_else(|| vec![ProcurementValueError::ArithmeticOverflow])?;
    let multiplier = checked_pow10(exponent)?;
    u128::from(amount.coefficient)
        .checked_mul(multiplier)
        .ok_or_else(|| vec![ProcurementValueError::ArithmeticOverflow])
}

fn checked_pow10(exponent: u8) -> Result<u128, Vec<ProcurementValueError>> {
    let mut value = 1_u128;
    for _ in 0..exponent {
        value = value
            .checked_mul(10)
            .ok_or_else(|| vec![ProcurementValueError::ArithmeticOverflow])?;
    }
    Ok(value)
}

fn checked_sum(values: impl IntoIterator<Item = u128>) -> Result<u128, Vec<ProcurementValueError>> {
    let mut total = 0_u128;
    for value in values {
        total = total
            .checked_add(value)
            .ok_or_else(|| vec![ProcurementValueError::ArithmeticOverflow])?;
    }
    Ok(total)
}

fn checked_sum_squares(
    values: impl IntoIterator<Item = u128>,
) -> Result<u128, Vec<ProcurementValueError>> {
    let mut total = 0_u128;
    for value in values {
        let square = value
            .checked_mul(value)
            .ok_or_else(|| vec![ProcurementValueError::ArithmeticOverflow])?;
        total = total
            .checked_add(square)
            .ok_or_else(|| vec![ProcurementValueError::ArithmeticOverflow])?;
    }
    Ok(total)
}

fn strict_provenance(provenance: &ProvenanceRef) -> bool {
    !provenance.source_ref.trim().is_empty()
        && provenance
            .content_hash
            .as_deref()
            .is_some_and(|hash| !hash.trim().is_empty())
}

fn valid_currency_code(currency: &str) -> bool {
    currency.len() == 3 && currency.bytes().all(|byte| byte.is_ascii_uppercase())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture_observation::{CaptureSubject, ConfidenceAssessment, ConfidenceLevel};
    use crate::institutional_graph::{
        AssertionStatus, DisclosureClass, InstitutionalNodeKind, InstitutionalNodeRef,
    };

    fn provenance(id: &str) -> ProvenanceRef {
        ProvenanceRef {
            source_ref: format!("source:{id}"),
            content_hash: Some(format!("sha256:{id}")),
        }
    }

    fn node(id: &str, kind: InstitutionalNodeKind) -> InstitutionalNodeRef {
        InstitutionalNodeRef { id: id.into(), kind }
    }

    fn award(id: &str, supplier: &str) -> InstitutionalEdge {
        InstitutionalEdge {
            id: id.into(),
            from: node(supplier, InstitutionalNodeKind::Organization),
            to: node("procedure", InstitutionalNodeKind::ProcurementProcedure),
            relation: InstitutionalRelationKind::ProcurementParticipation {
                role: ProcurementRole::Awardee,
            },
            disclosure: DisclosureClass::PublicMetadata,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![provenance(id)],
            challenge_refs: vec![],
            recorded_at: 10,
            valid_from: None,
            valid_until: None,
        }
    }

    fn record(
        edge: &str,
        award_ref: &str,
        coefficient: u64,
        scale: u8,
        currency: &str,
    ) -> SupplierAttributedAwardValue {
        SupplierAttributedAwardValue {
            value_ref: format!("value:{edge}"),
            award_edge_ref: edge.into(),
            award_ref: award_ref.into(),
            amount: ExactCurrencyAmount {
                coefficient,
                scale,
                currency: currency.into(),
            },
            provenance: vec![provenance(&format!("value:{edge}"))],
        }
    }

    fn value_set(records: Vec<SupplierAttributedAwardValue>) -> ProcurementAwardValueSet {
        ProcurementAwardValueSet {
            snapshot_ref: "values:snapshot:1".into(),
            value_semantics_ref: "ocds:1.1.5:award.value".into(),
            currency_registry_ref: "iso4217:2015".into(),
            currency_registry_provenance: vec![provenance("iso4217")],
            records,
        }
    }

    fn context() -> ObservationContext {
        ObservationContext {
            observation_id: "obs:value-weighted".into(),
            subject: CaptureSubject::ContractingProcedure("procedure".into()),
            observed_at: 100,
            confidence: ConfidenceAssessment::Qualitative {
                level: ConfidenceLevel::Moderate,
                basis: "fixture".into(),
            },
            limitations: vec!["fixture".into()],
        }
    }

    #[test]
    fn value_weighted_hhi_is_exact() {
        let edges = vec![award("a", "supplier-a"), award("b", "supplier-b")];
        let values = value_set(vec![
            record("a", "award-a", 100, 0, "USD"),
            record("b", "award-b", 300, 0, "USD"),
        ]);
        let result = ProcurementValueContract::value_weighted_supplier_concentration(
            &edges, &values, context(),
        )
        .unwrap();
        assert_eq!(result.derived.observation.measurement.value.numerator, 100_000);
        assert_eq!(result.derived.observation.measurement.value.denominator, 160_000);
    }

    #[test]
    fn decimal_scales_normalize_without_rounding() {
        let edges = vec![award("a", "supplier-a"), award("b", "supplier-b")];
        let values = value_set(vec![
            record("a", "award-a", 1000, 2, "USD"),
            record("b", "award-b", 500, 1, "USD"),
        ]);
        let result = ProcurementValueContract::value_weighted_supplier_concentration(
            &edges, &values, context(),
        )
        .unwrap();
        assert_eq!(result.normalized_scale, 2);
        assert_eq!(result.derived.observation.measurement.value.numerator, 26_000_000);
        assert_eq!(result.derived.observation.measurement.value.denominator, 36_000_000);
    }

    #[test]
    fn mixed_currency_fails_closed() {
        let values = value_set(vec![
            record("a", "award-a", 100, 0, "USD"),
            record("b", "award-b", 100, 0, "EUR"),
        ]);
        assert!(validate_value_set(&values)
            .unwrap_err()
            .contains(&ProcurementValueError::MixedCurrencies));
    }

    #[test]
    fn joint_award_total_cannot_be_duplicated_across_suppliers() {
        let values = value_set(vec![
            record("a", "joint-award", 100, 0, "USD"),
            record("b", "joint-award", 100, 0, "USD"),
        ]);
        assert!(validate_value_set(&values)
            .unwrap_err()
            .contains(&ProcurementValueError::DuplicateAwardReference {
                award_ref: "joint-award".into()
            }));
    }

    #[test]
    fn every_award_edge_requires_a_value() {
        let edges = vec![award("a", "supplier-a"), award("b", "supplier-b")];
        let values = value_set(vec![record("a", "award-a", 100, 0, "USD")]);
        assert!(ProcurementValueContract::value_weighted_supplier_concentration(
            &edges, &values, context(),
        )
        .unwrap_err()
        .contains(&ProcurementValueError::MissingValueForAwardEdge {
            award_edge_ref: "b".into()
        }));
    }
}
