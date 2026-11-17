// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure, deterministic reconciliation for food-lot provenance, private
//! household receipts and append-only local receipt events.
//!
//! This module has no Holochain calls and performs no persistence or mutation.
//! A UI/local application supplies already retrieved records; the function
//! returns a read-only reconciliation report. It is not a food-safety check.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const FOOD_LOT_RECONCILIATION_SCHEMA_VERSION: u32 = 1;
pub const FOOD_LOT_RECONCILIATION_CLAIM_CEILING: &str =
    "Read-only reconciliation of submitted lot, receipt, quantity and event declarations as of the stated time; not proof of physical inventory, freshness, food safety, allergen status, suitability, or authority to purchase, consume, transfer, discard or actuate.";

const JSON_SAFE_MAX: u128 = 9_007_199_254_740_991;
const NANO_PER_MILLIGRAM: u128 = 1_000_000;
const NANO_PER_GRAM: u128 = 1_000_000_000;
const NANO_PER_KILOGRAM: u128 = 1_000_000_000_000;
const NANO_LITRES_PER_MILLILITRE: u128 = 1_000_000;
const NANO_LITRES_PER_LITRE: u128 = 1_000_000_000;
const SCALE_MAX: u8 = 6;

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct FoodLotQuantityV1 {
    pub dimension: String,
    pub value: u64,
    pub decimal_places: u8,
    pub unit: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct FoodLotEvidenceV1 {
    pub source_ref: String,
    pub evidence_class: String,
    pub method_ref: Option<String>,
    pub calibration_ref: Option<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FoodLotProvenanceV1 {
    pub schema_version: u32,
    pub lot_ref: String,
    pub revision: u32,
    pub product_ref: String,
    pub producer_ref: String,
    pub supplier_lot_code: Option<String>,
    pub listing_ref: Option<String>,
    pub origin_ref: Option<String>,
    pub quantity: FoodLotQuantityV1,
    pub harvested_at: Option<u64>,
    pub packed_at: Option<u64>,
    pub best_before_at: Option<u64>,
    pub use_by_at: Option<u64>,
    pub handling_ref: Option<String>,
    pub evidence: FoodLotEvidenceV1,
    pub claim_ceiling: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FoodLotHouseholdReceiptV1 {
    pub schema_version: u32,
    pub receipt_id: String,
    pub product_lot_ref: String,
    pub source_order_ref: Option<String>,
    pub received_at: u64,
    pub receipt_state: String,
    pub received_quantity: FoodLotQuantityV1,
    pub accepted_quantity: Option<FoodLotQuantityV1>,
    pub rejected_quantity: Option<FoodLotQuantityV1>,
    pub privacy_scope: serde_json::Value,
    pub evidence: FoodLotEvidenceV1,
    pub claim_ceiling: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FoodLotHouseholdEventV1 {
    pub schema_version: u32,
    pub event_id: String,
    pub receipt_id: String,
    pub previous_event_ref: Option<String>,
    pub sequence_number: u64,
    pub event_kind: String,
    pub occurred_at: u64,
    pub quantity: Option<FoodLotQuantityV1>,
    pub child_receipt_id: Option<String>,
    pub privacy_scope: serde_json::Value,
    pub evidence: FoodLotEvidenceV1,
    pub claim_ceiling: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FoodLotReconciliationRequestV1 {
    pub schema_version: u32,
    pub reconciliation_id: String,
    pub ingredient_id: String,
    pub expected_product_ref: String,
    pub expected_lot_revision: u32,
    pub required_quantity: FoodLotQuantityV1,
    pub lot: Option<FoodLotProvenanceV1>,
    pub receipt: Option<FoodLotHouseholdReceiptV1>,
    pub events: Vec<FoodLotHouseholdEventV1>,
    /// Child receipt records referenced by this receipt's Split events.
    pub child_receipts: Vec<FoodLotHouseholdReceiptV1>,
    /// Explicit current-time input used for use-by and future-event checks.
    pub as_of: u64,
    pub claim_ceiling: String,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FoodLotReconciliationDispositionV1 {
    Resolved,
    Indeterminate,
    Conflicted,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FoodLotPlanningEligibilityV1 {
    Eligible,
    NotEligible,
    Unknown,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FoodLotReconciliationIssueCodeV1 {
    MissingLot,
    MissingReceipt,
    AmbiguousReceipt,
    QuarantinedReceipt,
    RejectedReceipt,
    StaleLotRevision,
    ProductMismatch,
    QuantityDimensionMismatch,
    QuantityUnitMismatch,
    ReceiptMassBalanceMismatch,
    EventSequenceGap,
    EventParentMismatch,
    DuplicateEvent,
    EventTimeRegression,
    Overconsumption,
    InvalidSplitBalance,
    RecallHold,
    DeclaredUseByElapsed,
    CurrentTimeUnknown,
    StaleOrMissingEvidence,
    UnsupportedExactConversion,
    InsufficientQuantity,
    InvalidQuantityRecord,
    InvalidClaimCeiling,
    MissingEventQuantity,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct FoodLotReconciliationReportV1 {
    pub schema_version: u32,
    pub reconciliation_id: String,
    pub ingredient_id: String,
    pub product_ref: String,
    pub product_lot_ref: Option<String>,
    pub lot_revision: Option<u32>,
    pub receipt_id: Option<String>,
    pub as_of: u64,
    pub disposition: FoodLotReconciliationDispositionV1,
    pub recipe_planning_eligibility: FoodLotPlanningEligibilityV1,
    pub required_quantity: FoodLotQuantityV1,
    pub available_quantity: Option<FoodLotQuantityV1>,
    pub event_chain_tip: Option<String>,
    pub issue_codes: Vec<FoodLotReconciliationIssueCodeV1>,
    pub evidence_refs: Vec<String>,
    pub claim_ceiling: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum QuantityBase {
    MassNanograms,
    VolumeNanolitres,
    Count,
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn issue(
    issues: &mut BTreeSet<FoodLotReconciliationIssueCodeV1>,
    code: FoodLotReconciliationIssueCodeV1,
) {
    issues.insert(code);
}

fn quantity_to_base(quantity: &FoodLotQuantityV1) -> Result<(QuantityBase, u128), FoodLotReconciliationIssueCodeV1> {
    if quantity.decimal_places > SCALE_MAX {
        return Err(FoodLotReconciliationIssueCodeV1::UnsupportedExactConversion);
    }
    let (base, dimension) = match quantity.unit.as_str() {
        "mg" => (NANO_PER_MILLIGRAM, QuantityBase::MassNanograms),
        "g" => (NANO_PER_GRAM, QuantityBase::MassNanograms),
        "kg" => (NANO_PER_KILOGRAM, QuantityBase::MassNanograms),
        "mL" => (NANO_LITRES_PER_MILLILITRE, QuantityBase::VolumeNanolitres),
        "L" => (NANO_LITRES_PER_LITRE, QuantityBase::VolumeNanolitres),
        "count" | "piece" => (1, QuantityBase::Count),
        _ => return Err(FoodLotReconciliationIssueCodeV1::QuantityUnitMismatch),
    };
    let expected_dimension = match dimension {
        QuantityBase::MassNanograms => "mass",
        QuantityBase::VolumeNanolitres => "volume",
        QuantityBase::Count => "count",
    };
    if quantity.dimension != expected_dimension {
        return Err(FoodLotReconciliationIssueCodeV1::QuantityDimensionMismatch);
    }
    let divisor = 10_u128
        .checked_pow(u32::from(quantity.decimal_places))
        .ok_or(FoodLotReconciliationIssueCodeV1::UnsupportedExactConversion)?;
    if base % divisor != 0 {
        return Err(FoodLotReconciliationIssueCodeV1::UnsupportedExactConversion);
    }
    let normalized = u128::from(quantity.value)
        .checked_mul(base / divisor)
        .ok_or(FoodLotReconciliationIssueCodeV1::UnsupportedExactConversion)?;
    if normalized > JSON_SAFE_MAX {
        return Err(FoodLotReconciliationIssueCodeV1::UnsupportedExactConversion);
    }
    Ok((dimension, normalized))
}

fn base_to_quantity(base_kind: QuantityBase, value: u128) -> Result<FoodLotQuantityV1, FoodLotReconciliationIssueCodeV1> {
    if value > JSON_SAFE_MAX {
        return Err(FoodLotReconciliationIssueCodeV1::UnsupportedExactConversion);
    }
    let (dimension, unit, decimal_places) = match base_kind {
        QuantityBase::MassNanograms => ("mass", "mg", 6),
        QuantityBase::VolumeNanolitres => ("volume", "mL", 6),
        QuantityBase::Count => ("count", "count", 0),
    };
    let exact_value = u64::try_from(value)
        .map_err(|_| FoodLotReconciliationIssueCodeV1::UnsupportedExactConversion)?;
    Ok(FoodLotQuantityV1 {
        dimension: dimension.to_owned(),
        value: exact_value,
        decimal_places,
        unit: unit.to_owned(),
    })
}

fn evidence_ref(
    evidence: &FoodLotEvidenceV1,
    refs: &mut BTreeSet<String>,
    issues: &mut BTreeSet<FoodLotReconciliationIssueCodeV1>,
) {
    if nonempty(&evidence.source_ref) {
        refs.insert(evidence.source_ref.clone());
    } else {
        issue(issues, FoodLotReconciliationIssueCodeV1::StaleOrMissingEvidence);
    }
    if matches!(
        evidence.evidence_class.as_str(),
        "Measured" | "IndependentlyAssayed" | "IndependentlyQualified"
    ) && evidence.calibration_ref.as_deref().is_none_or(|value| !nonempty(value))
    {
        issue(issues, FoodLotReconciliationIssueCodeV1::StaleOrMissingEvidence);
    }
    if matches!(
        evidence.evidence_class.as_str(),
        "IndependentlyAssayed" | "IndependentlyQualified"
    ) && evidence.method_ref.as_deref().is_none_or(|value| !nonempty(value))
    {
        issue(issues, FoodLotReconciliationIssueCodeV1::StaleOrMissingEvidence);
    }
    if let Some(method) = &evidence.method_ref {
        if nonempty(method) {
            refs.insert(method.clone());
        }
    }
    if let Some(calibration) = &evidence.calibration_ref {
        if nonempty(calibration) {
            refs.insert(calibration.clone());
        }
    }
}

fn is_conflict(code: FoodLotReconciliationIssueCodeV1) -> bool {
    matches!(
        code,
        FoodLotReconciliationIssueCodeV1::AmbiguousReceipt
            | FoodLotReconciliationIssueCodeV1::QuarantinedReceipt
            | FoodLotReconciliationIssueCodeV1::RejectedReceipt
            | FoodLotReconciliationIssueCodeV1::StaleLotRevision
            | FoodLotReconciliationIssueCodeV1::ProductMismatch
            | FoodLotReconciliationIssueCodeV1::ReceiptMassBalanceMismatch
            | FoodLotReconciliationIssueCodeV1::EventSequenceGap
            | FoodLotReconciliationIssueCodeV1::EventParentMismatch
            | FoodLotReconciliationIssueCodeV1::DuplicateEvent
            | FoodLotReconciliationIssueCodeV1::EventTimeRegression
            | FoodLotReconciliationIssueCodeV1::Overconsumption
            | FoodLotReconciliationIssueCodeV1::InvalidSplitBalance
            | FoodLotReconciliationIssueCodeV1::RecallHold
            | FoodLotReconciliationIssueCodeV1::DeclaredUseByElapsed
            | FoodLotReconciliationIssueCodeV1::InvalidQuantityRecord
            | FoodLotReconciliationIssueCodeV1::InvalidClaimCeiling
    )
}

fn validate_receipt_balance(
    receipt: &FoodLotHouseholdReceiptV1,
    issues: &mut BTreeSet<FoodLotReconciliationIssueCodeV1>,
) -> Option<(QuantityBase, u128)> {
    let Some(accepted) = &receipt.accepted_quantity else {
        return None;
    };
    let Some(rejected) = &receipt.rejected_quantity else {
        return None;
    };
    let (received_kind, received) = match quantity_to_base(&receipt.received_quantity) {
        Ok(value) => value,
        Err(code) => {
            issue(issues, code);
            return None;
        }
    };
    let (accepted_kind, accepted_value) = match quantity_to_base(accepted) {
        Ok(value) => value,
        Err(code) => {
            issue(issues, code);
            return None;
        }
    };
    let (rejected_kind, rejected_value) = match quantity_to_base(rejected) {
        Ok(value) => value,
        Err(code) => {
            issue(issues, code);
            return None;
        }
    };
    if received_kind != accepted_kind || received_kind != rejected_kind {
        issue(issues, FoodLotReconciliationIssueCodeV1::QuantityDimensionMismatch);
        return None;
    }
    let Some(sum) = accepted_value.checked_add(rejected_value) else {
        issue(issues, FoodLotReconciliationIssueCodeV1::UnsupportedExactConversion);
        return None;
    };
    if sum != received {
        issue(issues, FoodLotReconciliationIssueCodeV1::ReceiptMassBalanceMismatch);
        return None;
    }
    Some((accepted_kind, accepted_value))
}

fn reconcile_event_chain(
    receipt_id: &str,
    events: &[FoodLotHouseholdEventV1],
    as_of: u64,
    issues: &mut BTreeSet<FoodLotReconciliationIssueCodeV1>,
) -> (Vec<FoodLotHouseholdEventV1>, Option<String>) {
    let mut ordered = events.to_vec();
    ordered.sort_by_key(|event| event.sequence_number);
    let mut ids = BTreeSet::new();
    let mut sequences = BTreeSet::new();
    let mut previous_id: Option<String> = None;
    let mut previous_time = 0_u64;

    for (index, event) in ordered.iter().enumerate() {
        if !ids.insert(event.event_id.as_str()) || !sequences.insert(event.sequence_number) {
            issue(issues, FoodLotReconciliationIssueCodeV1::DuplicateEvent);
        }
        if u64::try_from(index).map_or(true, |expected| event.sequence_number != expected) {
            issue(issues, FoodLotReconciliationIssueCodeV1::EventSequenceGap);
        }
        if event.receipt_id != receipt_id {
            issue(issues, FoodLotReconciliationIssueCodeV1::EventParentMismatch);
        }
        if event.previous_event_ref != previous_id {
            issue(issues, FoodLotReconciliationIssueCodeV1::EventParentMismatch);
        }
        if event.occurred_at < previous_time || event.occurred_at > as_of {
            issue(issues, FoodLotReconciliationIssueCodeV1::EventTimeRegression);
        }
        previous_id = Some(event.event_id.clone());
        previous_time = event.occurred_at;
    }
    let tip = ordered.last().map(|event| event.event_id.clone());
    (ordered, tip)
}

fn reconcile_child_splits(
    parent_lot_ref: &str,
    split_events: &[FoodLotHouseholdEventV1],
    child_receipts: &[FoodLotHouseholdReceiptV1],
    parent_kind: QuantityBase,
    issues: &mut BTreeSet<FoodLotReconciliationIssueCodeV1>,
    evidence_refs: &mut BTreeSet<String>,
) -> Option<u128> {
    let mut seen_child_ids = BTreeSet::new();
    let mut total_split = 0_u128;
    for event in split_events {
        let Some(child_id) = event.child_receipt_id.as_deref() else {
            issue(issues, FoodLotReconciliationIssueCodeV1::InvalidSplitBalance);
            continue;
        };
        if !seen_child_ids.insert(child_id) {
            issue(issues, FoodLotReconciliationIssueCodeV1::InvalidSplitBalance);
            continue;
        }
        let Some(split_quantity) = event.quantity.as_ref() else {
            issue(issues, FoodLotReconciliationIssueCodeV1::MissingEventQuantity);
            continue;
        };
        let (split_kind, split_value) = match quantity_to_base(split_quantity) {
            Ok(value) => value,
            Err(code) => {
                issue(issues, code);
                continue;
            }
        };
        if split_kind != parent_kind {
            issue(issues, FoodLotReconciliationIssueCodeV1::QuantityDimensionMismatch);
            continue;
        }
        let Some(child) = child_receipts.iter().find(|receipt| receipt.receipt_id == child_id) else {
            issue(issues, FoodLotReconciliationIssueCodeV1::InvalidSplitBalance);
            continue;
        };
        if child.product_lot_ref != parent_lot_ref {
            issue(issues, FoodLotReconciliationIssueCodeV1::ProductMismatch);
            continue;
        }
        let Some(child_received) = quantity_to_base(&child.received_quantity).ok() else {
            issue(issues, FoodLotReconciliationIssueCodeV1::InvalidSplitBalance);
            continue;
        };
        if child_received.0 != split_kind || child_received.1 != split_value {
            issue(issues, FoodLotReconciliationIssueCodeV1::InvalidSplitBalance);
            continue;
        }
        let Some(_child_balance) = validate_receipt_balance(child, issues) else {
            issue(issues, FoodLotReconciliationIssueCodeV1::InvalidSplitBalance);
            continue;
        };
        evidence_ref(&child.evidence, evidence_refs, issues);
        total_split = match total_split.checked_add(split_value) {
            Some(sum) if sum <= JSON_SAFE_MAX => sum,
            _ => {
                issue(issues, FoodLotReconciliationIssueCodeV1::InvalidSplitBalance);
                return None;
            }
        };
    }
    Some(total_split)
}

/// Reconcile one proposed ingredient requirement with a canonical producer lot,
/// one private household receipt, its event chain and any child receipts
/// generated by split events. The function is deterministic and read-only.
pub fn reconcile_food_lot(
    request: &FoodLotReconciliationRequestV1,
) -> FoodLotReconciliationReportV1 {
    let mut issues = BTreeSet::new();
    let mut refs = BTreeSet::new();
    if request.as_of == 0 {
        issue(&mut issues, FoodLotReconciliationIssueCodeV1::CurrentTimeUnknown);
    }
    if request.schema_version != FOOD_LOT_RECONCILIATION_SCHEMA_VERSION
        || !nonempty(&request.reconciliation_id)
        || !nonempty(&request.ingredient_id)
        || !nonempty(&request.expected_product_ref)
    {
        issue(&mut issues, FoodLotReconciliationIssueCodeV1::InvalidQuantityRecord);
    }
    let required = quantity_to_base(&request.required_quantity);
    let Some(lot) = request.lot.as_ref() else {
        issue(&mut issues, FoodLotReconciliationIssueCodeV1::MissingLot);
        return report_from(request, None, None, None, issues, refs);
    };
    if lot.schema_version != 1
        || !nonempty(&lot.lot_ref)
        || !nonempty(&lot.product_ref)
        || lot.revision == 0
    {
        issue(&mut issues, FoodLotReconciliationIssueCodeV1::InvalidQuantityRecord);
    }
    evidence_ref(&lot.evidence, &mut refs, &mut issues);
    if lot.revision != request.expected_lot_revision {
        issue(&mut issues, FoodLotReconciliationIssueCodeV1::StaleLotRevision);
    }
    if lot.product_ref != request.expected_product_ref {
        issue(&mut issues, FoodLotReconciliationIssueCodeV1::ProductMismatch);
    }
    if request.as_of > 0 && lot.use_by_at.is_some_and(|use_by| request.as_of >= use_by) {
        issue(&mut issues, FoodLotReconciliationIssueCodeV1::DeclaredUseByElapsed);
    }
    let lot_ref = lot.lot_ref.clone();

    let Some(receipt) = request.receipt.as_ref() else {
        issue(&mut issues, FoodLotReconciliationIssueCodeV1::MissingReceipt);
        return report_from(request, Some(lot), None, None, issues, refs);
    };
    if receipt.schema_version != 1
        || !nonempty(&receipt.receipt_id)
        || receipt.product_lot_ref != lot_ref
    {
        issue(&mut issues, FoodLotReconciliationIssueCodeV1::ProductMismatch);
    }
    evidence_ref(&receipt.evidence, &mut refs, &mut issues);

    match receipt.receipt_state.as_str() {
        "ambiguous" => issue(&mut issues, FoodLotReconciliationIssueCodeV1::AmbiguousReceipt),
        "quarantined" => issue(&mut issues, FoodLotReconciliationIssueCodeV1::QuarantinedReceipt),
        "rejected" => issue(&mut issues, FoodLotReconciliationIssueCodeV1::RejectedReceipt),
        "accepted" | "partial" => {}
        _ => issue(&mut issues, FoodLotReconciliationIssueCodeV1::InvalidQuantityRecord),
    }

    let Some((base_kind, accepted_value)) = validate_receipt_balance(receipt, &mut issues) else {
        if !issues.iter().any(|code| matches!(
            code,
            FoodLotReconciliationIssueCodeV1::ReceiptMassBalanceMismatch
                | FoodLotReconciliationIssueCodeV1::QuantityDimensionMismatch
                | FoodLotReconciliationIssueCodeV1::QuantityUnitMismatch
                | FoodLotReconciliationIssueCodeV1::UnsupportedExactConversion
                | FoodLotReconciliationIssueCodeV1::InvalidQuantityRecord
        )) {
            issue(&mut issues, FoodLotReconciliationIssueCodeV1::InvalidQuantityRecord);
        }
        return report_from(request, Some(lot), Some(receipt), None, issues, refs);
    };

    let (ordered_events, event_tip) = reconcile_event_chain(
        &receipt.receipt_id,
        &request.events,
        request.as_of,
        &mut issues,
    );
    let mut consumed_or_discarded = 0_u128;
    let mut split_events = Vec::new();
    let mut recall_hold = false;

    for event in &ordered_events {
        evidence_ref(&event.evidence, &mut refs, &mut issues);
        match event.event_kind.as_str() {
            "Consumed" | "Discarded" => {
                let Some(quantity) = event.quantity.as_ref() else {
                    issue(&mut issues, FoodLotReconciliationIssueCodeV1::MissingEventQuantity);
                    continue;
                };
                let (kind, value) = match quantity_to_base(quantity) {
                    Ok(value) => value,
                    Err(code) => {
                        issue(&mut issues, code);
                        continue;
                    }
                };
                if kind != base_kind {
                    issue(&mut issues, FoodLotReconciliationIssueCodeV1::QuantityDimensionMismatch);
                    continue;
                }
                consumed_or_discarded = match consumed_or_discarded.checked_add(value) {
                    Some(sum) => sum,
                    None => {
                        issue(&mut issues, FoodLotReconciliationIssueCodeV1::Overconsumption);
                        accepted_value
                    }
                };
            }
            "Split" => split_events.push(event.clone()),
            "RecallNoticeReceived" => recall_hold = true,
            "Expired" => issue(&mut issues, FoodLotReconciliationIssueCodeV1::DeclaredUseByElapsed),
            "Opened" | "Reconciled" => {}
            _ => issue(&mut issues, FoodLotReconciliationIssueCodeV1::InvalidQuantityRecord),
        }
    }
    if recall_hold {
        issue(&mut issues, FoodLotReconciliationIssueCodeV1::RecallHold);
    }

    let split_total = reconcile_child_splits(
        &lot_ref,
        &split_events,
        &request.child_receipts,
        base_kind,
        &mut issues,
        &mut refs,
    )
    .unwrap_or(0);
    let already_allocated = consumed_or_discarded.checked_add(split_total);
    let remaining = match already_allocated {
        Some(used) if used <= accepted_value => Some(accepted_value - used),
        _ => {
            issue(&mut issues, FoodLotReconciliationIssueCodeV1::Overconsumption);
            None
        }
    };

    let available = match remaining {
        Some(value) => base_to_quantity(base_kind, value).ok(),
        None => None,
    };
    if required.is_err() {
        issue(&mut issues, required.unwrap_err());
    } else if let (Ok((required_kind, required_value)), Some(remaining)) = (required, remaining) {
        if required_kind != base_kind {
            issue(&mut issues, FoodLotReconciliationIssueCodeV1::QuantityDimensionMismatch);
        } else if remaining < required_value {
            issue(&mut issues, FoodLotReconciliationIssueCodeV1::InsufficientQuantity);
        }
    }

    report_from_with_tip(
        request,
        Some(lot),
        Some(receipt),
        available,
        event_tip,
        issues,
        refs,
    )
}

fn report_from(
    request: &FoodLotReconciliationRequestV1,
    lot: Option<&FoodLotProvenanceV1>,
    receipt: Option<&FoodLotHouseholdReceiptV1>,
    available: Option<FoodLotQuantityV1>,
    issues: BTreeSet<FoodLotReconciliationIssueCodeV1>,
    refs: BTreeSet<String>,
) -> FoodLotReconciliationReportV1 {
    report_from_with_tip(request, lot, receipt, available, None, issues, refs)
}

fn report_from_with_tip(
    request: &FoodLotReconciliationRequestV1,
    lot: Option<&FoodLotProvenanceV1>,
    receipt: Option<&FoodLotHouseholdReceiptV1>,
    available: Option<FoodLotQuantityV1>,
    event_tip: Option<String>,
    mut issues: BTreeSet<FoodLotReconciliationIssueCodeV1>,
    refs: BTreeSet<String>,
) -> FoodLotReconciliationReportV1 {
    if issues.contains(&FoodLotReconciliationIssueCodeV1::StaleOrMissingEvidence)
        && issues.iter().any(|code| is_conflict(*code))
    {
        // Preserve both findings; stale evidence does not erase a structural conflict.
    }
    let disposition = if issues.iter().any(|code| is_conflict(*code)) {
        FoodLotReconciliationDispositionV1::Conflicted
    } else if issues
        .iter()
        .any(|code| *code != FoodLotReconciliationIssueCodeV1::InsufficientQuantity)
    {
        FoodLotReconciliationDispositionV1::Indeterminate
    } else {
        FoodLotReconciliationDispositionV1::Resolved
    };
    let eligibility = match disposition {
        FoodLotReconciliationDispositionV1::Indeterminate => FoodLotPlanningEligibilityV1::Unknown,
        FoodLotReconciliationDispositionV1::Conflicted => FoodLotPlanningEligibilityV1::NotEligible,
        FoodLotReconciliationDispositionV1::Resolved => {
            if issues.contains(&FoodLotReconciliationIssueCodeV1::InsufficientQuantity) {
                FoodLotPlanningEligibilityV1::NotEligible
            } else {
                FoodLotPlanningEligibilityV1::Eligible
            }
        }
    };
    let mut issue_codes: Vec<_> = issues.iter().copied().collect();
    issue_codes.sort();
    FoodLotReconciliationReportV1 {
        schema_version: FOOD_LOT_RECONCILIATION_SCHEMA_VERSION,
        reconciliation_id: request.reconciliation_id.clone(),
        ingredient_id: request.ingredient_id.clone(),
        product_ref: lot.map(|value| value.product_ref.clone()).unwrap_or_else(|| request.expected_product_ref.clone()),
        product_lot_ref: lot.map(|value| value.lot_ref.clone()),
        lot_revision: lot.map(|value| value.revision),
        receipt_id: receipt.map(|value| value.receipt_id.clone()),
        as_of: request.as_of,
        disposition,
        recipe_planning_eligibility: eligibility,
        required_quantity: request.required_quantity.clone(),
        available_quantity: if disposition == FoodLotReconciliationDispositionV1::Resolved {
            available
        } else {
            None
        },
        event_chain_tip: event_tip,
        issue_codes,
        evidence_refs: refs.into_iter().collect(),
        claim_ceiling: FOOD_LOT_RECONCILIATION_CLAIM_CEILING.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn evidence() -> FoodLotEvidenceV1 {
        FoodLotEvidenceV1 {
            source_ref: "fixture:receipt-scan".to_owned(),
            evidence_class: "Measured".to_owned(),
            method_ref: Some("fixture:scale-method".to_owned()),
            calibration_ref: Some("fixture:scale-calibration".to_owned()),
        }
    }

    fn qty(value: u64, unit: &str) -> FoodLotQuantityV1 {
        let dimension = match unit {
            "mg" | "g" | "kg" => "mass",
            "mL" | "L" => "volume",
            "count" | "piece" => "count",
            _ => "unknown",
        };
        FoodLotQuantityV1 {
            dimension: dimension.to_owned(),
            value,
            decimal_places: 0,
            unit: unit.to_owned(),
        }
    }

    fn request() -> FoodLotReconciliationRequestV1 {
        FoodLotReconciliationRequestV1 {
            schema_version: 1,
            reconciliation_id: "fixture:reconcile-001".to_owned(),
            ingredient_id: "rice".to_owned(),
            expected_product_ref: "mycelix:product:rice-brown".to_owned(),
            expected_lot_revision: 2,
            required_quantity: qty(300, "g"),
            lot: Some(FoodLotProvenanceV1 {
                schema_version: 1,
                lot_ref: "mycelix:food-lot:rice-001".to_owned(),
                revision: 2,
                product_ref: "mycelix:product:rice-brown".to_owned(),
                producer_ref: "did:mycelix:producer:001".to_owned(),
                supplier_lot_code: Some("RL-2026-001".to_owned()),
                listing_ref: Some("mycelix:listing:001".to_owned()),
                origin_ref: None,
                quantity: qty(2, "kg"),
                harvested_at: Some(1_700_000_000),
                packed_at: Some(1_700_100_000),
                best_before_at: Some(1_800_000_000),
                use_by_at: Some(1_850_000_000),
                handling_ref: None,
                evidence: evidence(),
                claim_ceiling: "producer lot declaration only".to_owned(),
            }),
            receipt: Some(FoodLotHouseholdReceiptV1 {
                schema_version: 1,
                receipt_id: "receipt:local-001".to_owned(),
                product_lot_ref: "mycelix:food-lot:rice-001".to_owned(),
                source_order_ref: None,
                received_at: 1_710_000_000,
                receipt_state: "accepted".to_owned(),
                received_quantity: qty(2, "kg"),
                accepted_quantity: Some(qty(1_900, "g")),
                rejected_quantity: Some(qty(100, "g")),
                privacy_scope: serde_json::json!({"storage":"local_private"}),
                evidence: evidence(),
                claim_ceiling: "local receipt declaration only".to_owned(),
            }),
            events: vec![],
            child_receipts: vec![],
            as_of: 1_720_000_000,
            claim_ceiling: FOOD_LOT_RECONCILIATION_CLAIM_CEILING.to_owned(),
        }
    }

    fn event(
        id: &str,
        sequence: u64,
        previous: Option<&str>,
        kind: &str,
        at: u64,
        quantity: Option<FoodLotQuantityV1>,
        child: Option<&str>,
    ) -> FoodLotHouseholdEventV1 {
        FoodLotHouseholdEventV1 {
            schema_version: 1,
            event_id: id.to_owned(),
            receipt_id: "receipt:local-001".to_owned(),
            previous_event_ref: previous.map(str::to_owned),
            sequence_number: sequence,
            event_kind: kind.to_owned(),
            occurred_at: at,
            quantity,
            child_receipt_id: child.map(str::to_owned),
            privacy_scope: serde_json::json!({"storage":"local_private"}),
            evidence: evidence(),
            claim_ceiling: "local event declaration only".to_owned(),
        }
    }

    #[test]
    fn exact_accepted_lot_and_receipt_resolve_for_recipe_planning() {
        let report = reconcile_food_lot(&request());
        assert_eq!(report.disposition, FoodLotReconciliationDispositionV1::Resolved);
        assert_eq!(report.recipe_planning_eligibility, FoodLotPlanningEligibilityV1::Eligible);
        assert_eq!(report.available_quantity, Some(FoodLotQuantityV1 {
            dimension: "mass".to_owned(),
            value: 1_900_000_000_000,
            decimal_places: 6,
            unit: "mg".to_owned(),
        }));
    }

    #[test]
    fn supplier_listing_without_receipt_is_not_household_stock() {
        let mut request = request();
        request.receipt = None;
        let report = reconcile_food_lot(&request);
        assert_eq!(report.disposition, FoodLotReconciliationDispositionV1::Indeterminate);
        assert_eq!(report.recipe_planning_eligibility, FoodLotPlanningEligibilityV1::Unknown);
    }

    #[test]
    fn receipt_accepted_rejected_balance_must_match_received() {
        let mut request = request();
        request.receipt.as_mut().unwrap().rejected_quantity = Some(qty(50, "g"));
        let report = reconcile_food_lot(&request);
        assert_eq!(report.disposition, FoodLotReconciliationDispositionV1::Conflicted);
        assert!(report.issue_codes.contains(&FoodLotReconciliationIssueCodeV1::ReceiptMassBalanceMismatch));
    }

    #[test]
    fn mass_requirement_cannot_be_inferred_from_volume() {
        let mut request = request();
        request.receipt.as_mut().unwrap().accepted_quantity = Some(qty(1, "L"));
        request.receipt.as_mut().unwrap().rejected_quantity = Some(qty(1, "L"));
        request.receipt.as_mut().unwrap().received_quantity = qty(2, "L");
        let report = reconcile_food_lot(&request);
        assert_eq!(report.disposition, FoodLotReconciliationDispositionV1::Indeterminate);
        assert!(report.issue_codes.contains(&FoodLotReconciliationIssueCodeV1::QuantityDimensionMismatch));
    }

    #[test]
    fn consumed_events_reduce_available_quantity_and_preserve_chain_tip() {
        let mut request = request();
        request.events = vec![event(
            "event:open",
            0,
            None,
            "Opened",
            1_715_000_000,
            None,
            None,
        ), event(
            "event:consume",
            1,
            Some("event:open"),
            "Consumed",
            1_716_000_000,
            Some(qty(250, "g")),
            None,
        )];
        let report = reconcile_food_lot(&request);
        assert_eq!(report.disposition, FoodLotReconciliationDispositionV1::Resolved);
        assert_eq!(report.event_chain_tip.as_deref(), Some("event:consume"));
        assert_eq!(report.available_quantity, Some(FoodLotQuantityV1 {
            dimension: "mass".to_owned(),
            value: 1_650_000_000_000,
            decimal_places: 6,
            unit: "mg".to_owned(),
        }));
    }

    #[test]
    fn sequence_gap_and_wrong_parent_are_conflicted() {
        let mut request = request();
        request.events = vec![event(
            "event:open",
            0,
            None,
            "Opened",
            1_715_000_000,
            None,
            None,
        ), event(
            "event:consume",
            2,
            Some("event:wrong-parent"),
            "Consumed",
            1_716_000_000,
            Some(qty(10, "g")),
            None,
        )];
        let report = reconcile_food_lot(&request);
        assert_eq!(report.disposition, FoodLotReconciliationDispositionV1::Conflicted);
        assert!(report.issue_codes.contains(&FoodLotReconciliationIssueCodeV1::EventSequenceGap));
        assert!(report.issue_codes.contains(&FoodLotReconciliationIssueCodeV1::EventParentMismatch));
    }

    #[test]
    fn overconsumption_is_conflicted_not_clamped_to_zero() {
        let mut request = request();
        request.events = vec![event(
            "event:consume",
            0,
            None,
            "Consumed",
            1_715_000_000,
            Some(qty(2_000, "g")),
            None,
        )];
        let report = reconcile_food_lot(&request);
        assert_eq!(report.disposition, FoodLotReconciliationDispositionV1::Conflicted);
        assert!(report.issue_codes.contains(&FoodLotReconciliationIssueCodeV1::Overconsumption));
        assert!(report.available_quantity.is_none());
    }

    #[test]
    fn recall_and_use_by_holds_prevent_recipe_eligibility() {
        let mut request = request();
        request.events = vec![event(
            "event:recall",
            0,
            None,
            "RecallNoticeReceived",
            1_715_000_000,
            None,
            None,
        )];
        let report = reconcile_food_lot(&request);
        assert_eq!(report.recipe_planning_eligibility, FoodLotPlanningEligibilityV1::NotEligible);
        assert!(report.issue_codes.contains(&FoodLotReconciliationIssueCodeV1::RecallHold));

        let mut request = request();
        request.lot.as_mut().unwrap().use_by_at = Some(1_719_000_000);
        let report = reconcile_food_lot(&request);
        assert!(report.issue_codes.contains(&FoodLotReconciliationIssueCodeV1::DeclaredUseByElapsed));
    }

    #[test]
    fn ambiguous_and_quarantined_receipts_are_not_eligible() {
        let mut request = request();
        request.receipt.as_mut().unwrap().receipt_state = "ambiguous".to_owned();
        assert_eq!(
            reconcile_food_lot(&request).recipe_planning_eligibility,
            FoodLotPlanningEligibilityV1::NotEligible
        );

        let mut request = request();
        request.receipt.as_mut().unwrap().receipt_state = "quarantined".to_owned();
        assert_eq!(
            reconcile_food_lot(&request).recipe_planning_eligibility,
            FoodLotPlanningEligibilityV1::NotEligible
        );
    }

    #[test]
    fn insufficient_quantity_is_resolved_but_not_planning_eligible() {
        let mut request = request();
        request.required_quantity = qty(3, "kg");
        let report = reconcile_food_lot(&request);
        assert_eq!(report.disposition, FoodLotReconciliationDispositionV1::Resolved);
        assert_eq!(report.recipe_planning_eligibility, FoodLotPlanningEligibilityV1::NotEligible);
        assert!(report.issue_codes.contains(&FoodLotReconciliationIssueCodeV1::InsufficientQuantity));
    }

    #[test]
    fn expired_future_or_regressed_event_times_are_conflicted() {
        let mut request = request();
        request.events = vec![event(
            "event:open",
            0,
            None,
            "Opened",
            1_716_000_000,
            None,
            None,
        ), event(
            "event:consume",
            1,
            Some("event:open"),
            "Consumed",
            1_715_000_000,
            Some(qty(10, "g")),
            None,
        )];
        assert_eq!(
            reconcile_food_lot(&request).disposition,
            FoodLotReconciliationDispositionV1::Conflicted
        );

        let mut request = request();
        request.as_of = 1_710_000_000;
        request.events = vec![event(
            "event:open",
            0,
            None,
            "Opened",
            1_715_000_000,
            None,
            None,
        )];
        assert!(reconcile_food_lot(&request).issue_codes.contains(&FoodLotReconciliationIssueCodeV1::EventTimeRegression));
    }

    #[test]
    fn explicit_child_receipt_must_match_split_event_quantity() {
        let mut request = request();
        request.events = vec![event(
            "event:split",
            0,
            None,
            "Split",
            1_715_000_000,
            Some(qty(200, "g")),
            Some("receipt:child-01"),
        )];
        let mut child = request.receipt.as_ref().unwrap().clone();
        child.receipt_id = "receipt:child-01".to_owned();
        child.received_quantity = qty(150, "g");
        child.accepted_quantity = Some(qty(150, "g"));
        child.rejected_quantity = Some(qty(0, "g"));
        request.child_receipts = vec![child];
        let report = reconcile_food_lot(&request);
        assert_eq!(report.disposition, FoodLotReconciliationDispositionV1::Conflicted);
        assert!(report.issue_codes.contains(&FoodLotReconciliationIssueCodeV1::InvalidSplitBalance));
    }

    #[test]
    fn stale_revision_and_product_mismatch_are_conflicted() {
        let mut request = request();
        request.expected_lot_revision = 1;
        assert!(reconcile_food_lot(&request).issue_codes.contains(&FoodLotReconciliationIssueCodeV1::StaleLotRevision));

        let mut request = request();
        request.expected_product_ref = "mycelix:product:other".to_owned();
        assert!(reconcile_food_lot(&request).issue_codes.contains(&FoodLotReconciliationIssueCodeV1::ProductMismatch));
    }

    #[test]
    fn unknown_as_of_time_is_indeterminate() {
        let mut request = request();
        request.as_of = 0;
        let report = reconcile_food_lot(&request);
        assert_eq!(report.disposition, FoodLotReconciliationDispositionV1::Indeterminate);
        assert!(report.issue_codes.contains(&FoodLotReconciliationIssueCodeV1::CurrentTimeUnknown));
    }
}
