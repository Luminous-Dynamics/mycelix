//! Independent feasibility checker for multi-request procurement allocations.
//!
//! This module does not optimize, authorize, reserve, order, dispatch, or pay.
//! It checks a proposed joint allocation against one frozen input manifest and
//! reconciles request demand, budgets, offers, versioned capability mappings,
//! and shared capacity pools. Capacity is counted once per pool, not once per
//! offer referencing it. Caller-supplied evidence labels and digests are not
//! cryptographically authenticated by this module.

use std::collections::{BTreeMap, BTreeSet};

use super::{EvidenceOrigin, EvidenceStatus};

const EVALUATOR_PROFILE: &str = "mycelix:joint-capacity-validator:v1";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointRequest {
    pub request_id: String,
    pub capability_id: String,
    pub capability_profile_digest: String,
    pub unit: String,
    pub currency: String,
    pub required_quantity: u32,
    pub max_total_cost_minor: u64,
    pub deliver_by: u64,
    /// A policy input for downstream choice; this validator does not decide priority.
    pub priority_rank: u16,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointOffer {
    pub offer_id: String,
    pub supplier_id: String,
    pub offered_capability_id: String,
    pub offered_profile_digest: String,
    pub unit: String,
    pub currency: String,
    pub capacity_pool_id: String,
    /// Offer-local ceiling, independent of the shared pool ceiling.
    pub max_allocatable_quantity: u32,
    pub minimum_order_quantity: u32,
    /// Nonzero orders must be min + n * increment.
    pub order_increment: u32,
    /// Must be normalized to a comparable unit-cost basis for this currency.
    /// Fixed fees, transport charges, tax and FX conversion are not modeled here.
    pub unit_cost_minor: u64,
    pub arrives_at: u64,
    pub evidence_status: EvidenceStatus,
    pub evidence_origin: EvidenceOrigin,
    pub evidence_observed_at: u64,
    pub evidence_valid_until: u64,
}

/// One physical capacity ceiling for one explicitly named capacity/time bucket.
/// Every offer that references this ID consumes from this same ceiling.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SharedCapacityPool {
    pub capacity_pool_id: String,
    pub unit: String,
    pub time_bucket: String,
    pub capacity_quantity: u32,
    pub snapshot_digest: String,
    pub evidence_status: EvidenceStatus,
    pub evidence_origin: EvidenceOrigin,
    pub observed_at: u64,
    pub valid_until: u64,
}

/// Versioned, explicit qualification from a request capability profile to an offer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CapabilityMapping {
    pub mapping_id: String,
    pub mapping_digest: String,
    pub request_id: String,
    pub offer_id: String,
    pub requested_capability_id: String,
    pub requested_profile_digest: String,
    pub offered_capability_id: String,
    pub offered_profile_digest: String,
    pub evidence_status: EvidenceStatus,
    pub evidence_origin: EvidenceOrigin,
    pub observed_at: u64,
    pub valid_until: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointAssignment {
    pub request_id: String,
    pub offer_id: String,
    pub quantity: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointAllocationInput {
    pub evaluated_at: u64,
    /// Caller-supplied digest of the frozen input. This module does not recompute it.
    pub input_manifest_digest: String,
    pub requests: Vec<JointRequest>,
    pub offers: Vec<JointOffer>,
    pub capacity_pools: Vec<SharedCapacityPool>,
    pub capability_mappings: Vec<CapabilityMapping>,
    /// Exactly one row per request/offer pair; quantity is an integer in the declared unit.
    pub assignments: Vec<JointAssignment>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointAllocationError {
    InvalidManifest,
    DuplicateRequestId,
    InvalidRequest,
    DuplicateOfferId,
    InvalidOffer,
    OfferEvidenceNotVerified,
    OfferEvidenceNotLive,
    OfferEvidenceFromFuture,
    OfferEvidenceStale,
    DuplicateCapacityPoolId,
    InvalidCapacityPool,
    PoolEvidenceNotVerified,
    PoolEvidenceNotLive,
    PoolEvidenceFromFuture,
    PoolEvidenceStale,
    DuplicateMappingId,
    DuplicateMappingForRequestOffer,
    MappingReferencesUnknownEntity,
    CapabilityMappingMismatch,
    MappingEvidenceNotVerified,
    MappingEvidenceNotLive,
    MappingEvidenceFromFuture,
    MappingEvidenceStale,
    DuplicateAssignment,
    ZeroQuantity,
    UnknownRequest,
    UnknownOffer,
    MissingCapacityPool,
    MissingCapabilityMapping,
    UnitMismatch,
    CurrencyMismatch,
    DeadlineMiss,
    DemandExceeded,
    OfferCapacityExceeded,
    CapacityPoolExceeded,
    InvalidOrderLot,
    RequestBudgetExceeded,
    ArithmeticOverflow,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RequestReconciliation {
    pub request_id: String,
    pub priority_rank: u16,
    pub unit: String,
    pub currency: String,
    pub required_quantity: u32,
    pub served_quantity: u64,
    pub unfilled_quantity: u64,
    pub cost_minor: u64,
    pub max_total_cost_minor: u64,
    pub fully_served: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OfferReconciliation {
    pub offer_id: String,
    pub capacity_pool_id: String,
    pub max_allocatable_quantity: u32,
    pub allocated_quantity: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PoolReconciliation {
    pub capacity_pool_id: String,
    pub unit: String,
    pub time_bucket: String,
    pub capacity_quantity: u32,
    pub allocated_quantity: u64,
    pub remaining_quantity: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnitQuantitySummary {
    pub capability_id: String,
    pub capability_profile_digest: String,
    pub unit: String,
    pub required_quantity: u64,
    pub served_quantity: u64,
    pub unfilled_quantity: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CurrencyCostSummary {
    pub currency: String,
    pub allocated_cost_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AllocationWitness {
    pub request_id: String,
    pub request_priority_rank: u16,
    pub request_capability_id: String,
    pub request_profile_digest: String,
    pub offer_id: String,
    pub supplier_id: String,
    pub capability_mapping_id: String,
    pub capability_mapping_digest: String,
    pub capacity_pool_id: String,
    pub capacity_snapshot_digest: String,
    pub time_bucket: String,
    pub quantity: u32,
    pub unit: String,
    pub currency: String,
    pub unit_cost_minor: u64,
    pub line_cost_minor: u64,
    pub arrives_at: u64,
    pub deliver_by: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointAllocationReport {
    pub evaluator_profile: &'static str,
    pub input_manifest_digest: String,
    pub evaluated_at: u64,
    pub request_count: usize,
    pub offer_count: usize,
    pub capacity_pool_count: usize,
    pub assignment_count: usize,
    /// Canonical line-level witness for every accepted assignment, including the exact
    /// mapping and capacity snapshot identifiers used to validate it.
    pub assignments: Vec<AllocationWitness>,
    pub requests: Vec<RequestReconciliation>,
    pub offers: Vec<OfferReconciliation>,
    pub capacity_pools: Vec<PoolReconciliation>,
    /// Quantities are aggregated only within an identical unit label.
    pub quantities_by_unit: Vec<UnitQuantitySummary>,
    /// Money is aggregated only within an identical currency label.
    pub costs_by_currency: Vec<CurrencyCostSummary>,
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn evidence_is_current(
    status: EvidenceStatus,
    origin: EvidenceOrigin,
    observed_at: u64,
    valid_until: u64,
    evaluated_at: u64,
) -> Result<(), JointAllocationError> {
    if status != EvidenceStatus::Verified {
        return Err(JointAllocationError::OfferEvidenceNotVerified);
    }
    if origin != EvidenceOrigin::LiveDomain {
        return Err(JointAllocationError::OfferEvidenceNotLive);
    }
    if observed_at > evaluated_at {
        return Err(JointAllocationError::OfferEvidenceFromFuture);
    }
    if valid_until < evaluated_at {
        return Err(JointAllocationError::OfferEvidenceStale);
    }
    Ok(())
}

/// Validate and reconcile a proposed joint assignment. This is a feasibility
/// gate, not an optimizer: it never changes the supplied assignments or chooses
/// a winner. All output collections are canonicalized by stable string IDs so
/// input vector order cannot change the report.
pub fn validate_joint_allocation(
    input: &JointAllocationInput,
) -> Result<JointAllocationReport, JointAllocationError> {
    if !non_empty(&input.input_manifest_digest) {
        return Err(JointAllocationError::InvalidManifest);
    }

    let mut requests: BTreeMap<String, &JointRequest> = BTreeMap::new();
    for request in &input.requests {
        if [
            request.request_id.as_str(),
            request.capability_id.as_str(),
            request.capability_profile_digest.as_str(),
            request.unit.as_str(),
            request.currency.as_str(),
        ]
        .iter()
        .any(|field| !non_empty(field))
            || request.required_quantity == 0
        {
            return Err(JointAllocationError::InvalidRequest);
        }
        if requests.insert(request.request_id.clone(), request).is_some() {
            return Err(JointAllocationError::DuplicateRequestId);
        }
    }

    let mut pools: BTreeMap<String, &SharedCapacityPool> = BTreeMap::new();
    for pool in &input.capacity_pools {
        if [
            pool.capacity_pool_id.as_str(),
            pool.unit.as_str(),
            pool.time_bucket.as_str(),
            pool.snapshot_digest.as_str(),
        ]
        .iter()
        .any(|field| !non_empty(field))
        {
            return Err(JointAllocationError::InvalidCapacityPool);
        }
        if pools.insert(pool.capacity_pool_id.clone(), pool).is_some() {
            return Err(JointAllocationError::DuplicateCapacityPoolId);
        }
        if pool.evidence_status != EvidenceStatus::Verified {
            return Err(JointAllocationError::PoolEvidenceNotVerified);
        }
        if pool.evidence_origin != EvidenceOrigin::LiveDomain {
            return Err(JointAllocationError::PoolEvidenceNotLive);
        }
        if pool.observed_at > input.evaluated_at {
            return Err(JointAllocationError::PoolEvidenceFromFuture);
        }
        if pool.valid_until < input.evaluated_at {
            return Err(JointAllocationError::PoolEvidenceStale);
        }
    }

    let mut offers: BTreeMap<String, &JointOffer> = BTreeMap::new();
    for offer in &input.offers {
        if [
            offer.offer_id.as_str(),
            offer.supplier_id.as_str(),
            offer.offered_capability_id.as_str(),
            offer.offered_profile_digest.as_str(),
            offer.unit.as_str(),
            offer.currency.as_str(),
            offer.capacity_pool_id.as_str(),
        ]
        .iter()
        .any(|field| !non_empty(field))
            || offer.max_allocatable_quantity == 0
            || offer.minimum_order_quantity == 0
            || offer.order_increment == 0
            || offer.minimum_order_quantity > offer.max_allocatable_quantity
        {
            return Err(JointAllocationError::InvalidOffer);
        }
        if offers.insert(offer.offer_id.clone(), offer).is_some() {
            return Err(JointAllocationError::DuplicateOfferId);
        }
        let pool = pools
            .get(&offer.capacity_pool_id)
            .ok_or(JointAllocationError::MissingCapacityPool)?;
        if pool.unit != offer.unit {
            return Err(JointAllocationError::UnitMismatch);
        }
        evidence_is_current(
            offer.evidence_status,
            offer.evidence_origin,
            offer.evidence_observed_at,
            offer.evidence_valid_until,
            input.evaluated_at,
        )?;
    }

    let mut mapping_ids = BTreeSet::new();
    let mut mappings: BTreeMap<(String, String), &CapabilityMapping> = BTreeMap::new();
    for mapping in &input.capability_mappings {
        if [
            mapping.mapping_id.as_str(),
            mapping.mapping_digest.as_str(),
            mapping.request_id.as_str(),
            mapping.offer_id.as_str(),
            mapping.requested_capability_id.as_str(),
            mapping.requested_profile_digest.as_str(),
            mapping.offered_capability_id.as_str(),
            mapping.offered_profile_digest.as_str(),
        ]
        .iter()
        .any(|field| !non_empty(field))
        {
            return Err(JointAllocationError::CapabilityMappingMismatch);
        }
        if !mapping_ids.insert(mapping.mapping_id.clone()) {
            return Err(JointAllocationError::DuplicateMappingId);
        }
        let request = requests
            .get(&mapping.request_id)
            .ok_or(JointAllocationError::MappingReferencesUnknownEntity)?;
        let offer = offers
            .get(&mapping.offer_id)
            .ok_or(JointAllocationError::MappingReferencesUnknownEntity)?;
        if mapping.requested_capability_id != request.capability_id
            || mapping.requested_profile_digest != request.capability_profile_digest
            || mapping.offered_capability_id != offer.offered_capability_id
            || mapping.offered_profile_digest != offer.offered_profile_digest
        {
            return Err(JointAllocationError::CapabilityMappingMismatch);
        }
        let key = (mapping.request_id.clone(), mapping.offer_id.clone());
        if mappings.insert(key, mapping).is_some() {
            return Err(JointAllocationError::DuplicateMappingForRequestOffer);
        }
    }

    let mut request_quantity: BTreeMap<String, u64> = BTreeMap::new();
    let mut request_cost: BTreeMap<String, u64> = BTreeMap::new();
    let mut offer_quantity: BTreeMap<String, u64> = BTreeMap::new();
    let mut pool_quantity: BTreeMap<String, u64> = BTreeMap::new();
    let mut seen_assignments = BTreeSet::new();

    for assignment in &input.assignments {
        if assignment.quantity == 0 {
            return Err(JointAllocationError::ZeroQuantity);
        }
        let pair = (assignment.request_id.clone(), assignment.offer_id.clone());
        if !seen_assignments.insert(pair.clone()) {
            return Err(JointAllocationError::DuplicateAssignment);
        }
        let request = requests
            .get(&assignment.request_id)
            .ok_or(JointAllocationError::UnknownRequest)?;
        let offer = offers
            .get(&assignment.offer_id)
            .ok_or(JointAllocationError::UnknownOffer)?;

        if request.unit != offer.unit {
            return Err(JointAllocationError::UnitMismatch);
        }
        if request.currency != offer.currency {
            return Err(JointAllocationError::CurrencyMismatch);
        }
        let pool = pools
            .get(&offer.capacity_pool_id)
            .ok_or(JointAllocationError::MissingCapacityPool)?;
        if pool.unit != offer.unit {
            return Err(JointAllocationError::UnitMismatch);
        }
        if offer.arrives_at > request.deliver_by {
            return Err(JointAllocationError::DeadlineMiss);
        }
        let mapping = mappings
            .get(&pair)
            .ok_or(JointAllocationError::MissingCapabilityMapping)?;
        if mapping.evidence_status != EvidenceStatus::Verified {
            return Err(JointAllocationError::MappingEvidenceNotVerified);
        }
        if mapping.evidence_origin != EvidenceOrigin::LiveDomain {
            return Err(JointAllocationError::MappingEvidenceNotLive);
        }
        if mapping.observed_at > input.evaluated_at {
            return Err(JointAllocationError::MappingEvidenceFromFuture);
        }
        if mapping.valid_until < input.evaluated_at {
            return Err(JointAllocationError::MappingEvidenceStale);
        }

        let request_served = request_quantity.entry(assignment.request_id.clone()).or_default();
        *request_served = request_served
            .checked_add(u64::from(assignment.quantity))
            .ok_or(JointAllocationError::ArithmeticOverflow)?;
        if *request_served > u64::from(request.required_quantity) {
            return Err(JointAllocationError::DemandExceeded);
        }

        let line_cost = offer
            .unit_cost_minor
            .checked_mul(u64::from(assignment.quantity))
            .ok_or(JointAllocationError::ArithmeticOverflow)?;
        let cost = request_cost.entry(assignment.request_id.clone()).or_default();
        *cost = cost
            .checked_add(line_cost)
            .ok_or(JointAllocationError::ArithmeticOverflow)?;
        if *cost > request.max_total_cost_minor {
            return Err(JointAllocationError::RequestBudgetExceeded);
        }

        let used_by_offer = offer_quantity.entry(assignment.offer_id.clone()).or_default();
        *used_by_offer = used_by_offer
            .checked_add(u64::from(assignment.quantity))
            .ok_or(JointAllocationError::ArithmeticOverflow)?;
        if *used_by_offer > u64::from(offer.max_allocatable_quantity) {
            return Err(JointAllocationError::OfferCapacityExceeded);
        }

        let used_by_pool = pool_quantity.entry(offer.capacity_pool_id.clone()).or_default();
        *used_by_pool = used_by_pool
            .checked_add(u64::from(assignment.quantity))
            .ok_or(JointAllocationError::ArithmeticOverflow)?;
        if *used_by_pool > u64::from(pool.capacity_quantity) {
            return Err(JointAllocationError::CapacityPoolExceeded);
        }
    }

    for (offer_id, offer) in &offers {
        let quantity = offer_quantity.get(offer_id).copied().unwrap_or(0);
        if quantity == 0 {
            continue;
        }
        if quantity < u64::from(offer.minimum_order_quantity)
            || (quantity - u64::from(offer.minimum_order_quantity))
                % u64::from(offer.order_increment)
                != 0
        {
            return Err(JointAllocationError::InvalidOrderLot);
        }
    }

    let mut request_reports = Vec::with_capacity(requests.len());
    let mut unit_totals: BTreeMap<(String, String, String), (u64, u64)> = BTreeMap::new();
    let mut currency_totals: BTreeMap<String, u64> = BTreeMap::new();

    for (request_id, request) in &requests {
        let served = request_quantity.get(request_id).copied().unwrap_or(0);
        let cost = request_cost.get(request_id).copied().unwrap_or(0);
        let unfilled = u64::from(request.required_quantity) - served;
        let unit_key = (
            request.capability_id.clone(),
            request.capability_profile_digest.clone(),
            request.unit.clone(),
        );
        let unit_total = unit_totals.entry(unit_key).or_default();
        unit_total.0 = unit_total
            .0
            .checked_add(u64::from(request.required_quantity))
            .ok_or(JointAllocationError::ArithmeticOverflow)?;
        unit_total.1 = unit_total
            .1
            .checked_add(served)
            .ok_or(JointAllocationError::ArithmeticOverflow)?;
        let currency_total = currency_totals.entry(request.currency.clone()).or_default();
        *currency_total = currency_total
            .checked_add(cost)
            .ok_or(JointAllocationError::ArithmeticOverflow)?;

        request_reports.push(RequestReconciliation {
            request_id: request_id.clone(),
            priority_rank: request.priority_rank,
            unit: request.unit.clone(),
            currency: request.currency.clone(),
            required_quantity: request.required_quantity,
            served_quantity: served,
            unfilled_quantity: unfilled,
            cost_minor: cost,
            max_total_cost_minor: request.max_total_cost_minor,
            fully_served: unfilled == 0,
        });
    }

    let mut offer_reports = Vec::with_capacity(offers.len());
    for (offer_id, offer) in &offers {
        offer_reports.push(OfferReconciliation {
            offer_id: offer_id.clone(),
            capacity_pool_id: offer.capacity_pool_id.clone(),
            max_allocatable_quantity: offer.max_allocatable_quantity,
            allocated_quantity: offer_quantity.get(offer_id).copied().unwrap_or(0),
        });
    }

    let mut pool_reports = Vec::with_capacity(pools.len());
    for (pool_id, pool) in &pools {
        let used = pool_quantity.get(pool_id).copied().unwrap_or(0);
        pool_reports.push(PoolReconciliation {
            capacity_pool_id: pool_id.clone(),
            unit: pool.unit.clone(),
            time_bucket: pool.time_bucket.clone(),
            capacity_quantity: pool.capacity_quantity,
            allocated_quantity: used,
            remaining_quantity: u64::from(pool.capacity_quantity) - used,
        });
    }

    let quantities_by_unit = unit_totals
        .into_iter()
        .map(|((capability_id, capability_profile_digest, unit), (required_quantity, served_quantity))| UnitQuantitySummary {
            capability_id,
            capability_profile_digest,
            unit,
            required_quantity,
            served_quantity,
            unfilled_quantity: required_quantity - served_quantity,
        })
        .collect();
    let costs_by_currency = currency_totals
        .into_iter()
        .map(|(currency, allocated_cost_minor)| CurrencyCostSummary {
            currency,
            allocated_cost_minor,
        })
        .collect();

    let mut assignment_witnesses = Vec::with_capacity(input.assignments.len());
    for assignment in &input.assignments {
        let request = requests
            .get(&assignment.request_id)
            .ok_or(JointAllocationError::UnknownRequest)?;
        let offer = offers
            .get(&assignment.offer_id)
            .ok_or(JointAllocationError::UnknownOffer)?;
        let pool = pools
            .get(&offer.capacity_pool_id)
            .ok_or(JointAllocationError::MissingCapacityPool)?;
        let pair = (assignment.request_id.clone(), assignment.offer_id.clone());
        let mapping = mappings
            .get(&pair)
            .ok_or(JointAllocationError::MissingCapabilityMapping)?;
        let line_cost_minor = offer
            .unit_cost_minor
            .checked_mul(u64::from(assignment.quantity))
            .ok_or(JointAllocationError::ArithmeticOverflow)?;
        assignment_witnesses.push(AllocationWitness {
            request_id: assignment.request_id.clone(),
            request_priority_rank: request.priority_rank,
            request_capability_id: request.capability_id.clone(),
            request_profile_digest: request.capability_profile_digest.clone(),
            offer_id: offer.offer_id.clone(),
            supplier_id: offer.supplier_id.clone(),
            capability_mapping_id: mapping.mapping_id.clone(),
            capability_mapping_digest: mapping.mapping_digest.clone(),
            capacity_pool_id: pool.capacity_pool_id.clone(),
            capacity_snapshot_digest: pool.snapshot_digest.clone(),
            time_bucket: pool.time_bucket.clone(),
            quantity: assignment.quantity,
            unit: request.unit.clone(),
            currency: request.currency.clone(),
            unit_cost_minor: offer.unit_cost_minor,
            line_cost_minor,
            arrives_at: offer.arrives_at,
            deliver_by: request.deliver_by,
        });
    }
    assignment_witnesses.sort_by(|left, right| {
        (&left.request_id, &left.offer_id).cmp(&(&right.request_id, &right.offer_id))
    });

    Ok(JointAllocationReport {
        evaluator_profile: EVALUATOR_PROFILE,
        input_manifest_digest: input.input_manifest_digest.clone(),
        evaluated_at: input.evaluated_at,
        request_count: requests.len(),
        offer_count: offers.len(),
        capacity_pool_count: pools.len(),
        assignment_count: input.assignments.len(),
        assignments: assignment_witnesses,
        requests: request_reports,
        offers: offer_reports,
        capacity_pools: pool_reports,
        quantities_by_unit,
        costs_by_currency,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        CapabilityMapping, JointAllocationError, JointAllocationInput, JointAssignment,
        JointOffer, JointRequest, SharedCapacityPool, validate_joint_allocation,
    };
    use crate::{EvidenceOrigin, EvidenceStatus};

    fn request(id: &str, quantity: u32, budget: u64) -> JointRequest {
        JointRequest {
            request_id: id.to_owned(),
            capability_id: "repair-kit".to_owned(),
            capability_profile_digest: "repair-kit-profile-v1".to_owned(),
            unit: "kit".to_owned(),
            currency: "USD".to_owned(),
            required_quantity: quantity,
            max_total_cost_minor: budget,
            deliver_by: 500,
            priority_rank: if id == "urgent" { 1 } else { 10 },
        }
    }

    fn offer(id: &str, pool: &str, cap: u32, cost: u64) -> JointOffer {
        JointOffer {
            offer_id: id.to_owned(),
            supplier_id: format!("supplier-{id}"),
            offered_capability_id: "repair-kit".to_owned(),
            offered_profile_digest: "repair-kit-profile-v1".to_owned(),
            unit: "kit".to_owned(),
            currency: "USD".to_owned(),
            capacity_pool_id: pool.to_owned(),
            max_allocatable_quantity: cap,
            minimum_order_quantity: 1,
            order_increment: 1,
            unit_cost_minor: cost,
            arrives_at: 100,
            evidence_status: EvidenceStatus::Verified,
            evidence_origin: EvidenceOrigin::LiveDomain,
            evidence_observed_at: 10,
            evidence_valid_until: 1_000,
        }
    }

    fn pool(id: &str, quantity: u32) -> SharedCapacityPool {
        SharedCapacityPool {
            capacity_pool_id: id.to_owned(),
            unit: "kit".to_owned(),
            time_bucket: "week-2026-44".to_owned(),
            capacity_quantity: quantity,
            snapshot_digest: format!("sha256-{id}"),
            evidence_status: EvidenceStatus::Verified,
            evidence_origin: EvidenceOrigin::LiveDomain,
            observed_at: 10,
            valid_until: 1_000,
        }
    }

    fn mapping(request_id: &str, offer_id: &str) -> CapabilityMapping {
        CapabilityMapping {
            mapping_id: format!("map-{request_id}-{offer_id}"),
            mapping_digest: format!("sha256-map-{request_id}-{offer_id}"),
            request_id: request_id.to_owned(),
            offer_id: offer_id.to_owned(),
            requested_capability_id: "repair-kit".to_owned(),
            requested_profile_digest: "repair-kit-profile-v1".to_owned(),
            offered_capability_id: "repair-kit".to_owned(),
            offered_profile_digest: "repair-kit-profile-v1".to_owned(),
            evidence_status: EvidenceStatus::Verified,
            evidence_origin: EvidenceOrigin::LiveDomain,
            observed_at: 10,
            valid_until: 1_000,
        }
    }

    fn fixture() -> JointAllocationInput {
        JointAllocationInput {
            evaluated_at: 20,
            input_manifest_digest: "sha256-joint-fixture-v1".to_owned(),
            requests: vec![request("urgent", 2, 200), request("routine", 3, 300)],
            offers: vec![offer("offer-a", "pool-a", 2, 50), offer("offer-b", "pool-b", 3, 60)],
            capacity_pools: vec![pool("pool-a", 2), pool("pool-b", 3)],
            capability_mappings: vec![
                mapping("urgent", "offer-a"),
                mapping("routine", "offer-b"),
            ],
            assignments: vec![
                JointAssignment { request_id: "urgent".to_owned(), offer_id: "offer-a".to_owned(), quantity: 2 },
                JointAssignment { request_id: "routine".to_owned(), offer_id: "offer-b".to_owned(), quantity: 3 },
            ],
        }
    }

    #[test]
    fn valid_joint_allocation_reconciles_each_request_pool_unit_and_currency() {
        let report = validate_joint_allocation(&fixture()).expect("valid allocation");
        assert_eq!(report.request_count, 2);
        assert_eq!(report.assignment_count, 2);
        assert_eq!(report.requests[0].request_id, "routine");
        assert_eq!(report.requests[0].served_quantity, 3);
        assert_eq!(report.requests[0].cost_minor, 180);
        assert_eq!(report.requests[1].request_id, "urgent");
        assert_eq!(report.requests[1].cost_minor, 100);
        assert_eq!(report.quantities_by_unit[0].required_quantity, 5);
        assert_eq!(report.quantities_by_unit[0].served_quantity, 5);
        assert_eq!(report.costs_by_currency[0].allocated_cost_minor, 280);
        assert_eq!(report.capacity_pools[0].remaining_quantity, 0);
        assert_eq!(report.assignments.len(), 2);
        assert_eq!(report.assignments[0].request_id, "routine");
        assert_eq!(report.assignments[0].capability_mapping_id, "map-routine-offer-b");
        assert_eq!(report.assignments[0].capacity_snapshot_digest, "sha256-pool-b");
        assert_eq!(report.assignments[0].line_cost_minor, 180);
    }

    #[test]
    fn two_offers_do_not_each_get_to_spend_the_full_shared_pool() {
        let mut input = fixture();
        input.offers = vec![offer("offer-a", "shared", 3, 10), offer("offer-b", "shared", 3, 20)];
        input.capacity_pools = vec![pool("shared", 4)];
        input.requests = vec![request("urgent", 3, 100), request("routine", 3, 100)];
        input.capability_mappings = vec![mapping("urgent", "offer-a"), mapping("routine", "offer-b")];
        input.assignments = vec![
            JointAssignment { request_id: "urgent".to_owned(), offer_id: "offer-a".to_owned(), quantity: 3 },
            JointAssignment { request_id: "routine".to_owned(), offer_id: "offer-b".to_owned(), quantity: 2 },
        ];
        assert_eq!(
            validate_joint_allocation(&input),
            Err(JointAllocationError::CapacityPoolExceeded)
        );
    }

    #[test]
    fn two_requests_cannot_both_claim_the_last_unit() {
        let mut input = fixture();
        input.offers = vec![offer("offer-a", "shared", 1, 10), offer("offer-b", "shared", 1, 20)];
        input.capacity_pools = vec![pool("shared", 1)];
        input.requests = vec![request("urgent", 1, 100), request("routine", 1, 100)];
        input.capability_mappings = vec![mapping("urgent", "offer-a"), mapping("routine", "offer-b")];
        input.assignments = vec![
            JointAssignment { request_id: "urgent".to_owned(), offer_id: "offer-a".to_owned(), quantity: 1 },
            JointAssignment { request_id: "routine".to_owned(), offer_id: "offer-b".to_owned(), quantity: 1 },
        ];
        assert_eq!(
            validate_joint_allocation(&input),
            Err(JointAllocationError::CapacityPoolExceeded)
        );
    }

    #[test]
    fn reports_unfilled_demand_without_promoting_partial_supply_to_fulfilled() {
        let mut input = fixture();
        input.assignments = vec![JointAssignment {
            request_id: "urgent".to_owned(),
            offer_id: "offer-a".to_owned(),
            quantity: 1,
        }];
        let report = validate_joint_allocation(&input).expect("partial allocation is a valid plan");
        let urgent = report.requests.iter().find(|r| r.request_id == "urgent").unwrap();
        assert_eq!(urgent.served_quantity, 1);
        assert_eq!(urgent.unfilled_quantity, 1);
        assert!(!urgent.fully_served);
        assert_eq!(report.quantities_by_unit[0].unfilled_quantity, 4);
    }

    #[test]
    fn stale_capacity_snapshot_fails_closed() {
        let mut input = fixture();
        input.capacity_pools[0].valid_until = 19;
        assert_eq!(
            validate_joint_allocation(&input),
            Err(JointAllocationError::PoolEvidenceStale)
        );
    }

    #[test]
    fn verified_synthetic_capacity_is_not_live_capacity() {
        let mut input = fixture();
        input.capacity_pools[0].evidence_origin = EvidenceOrigin::SyntheticSimulation;
        assert_eq!(
            validate_joint_allocation(&input),
            Err(JointAllocationError::PoolEvidenceNotLive)
        );
    }

    #[test]
    fn stale_capability_mapping_cannot_qualify_an_assignment() {
        let mut input = fixture();
        input.capability_mappings[0].valid_until = 19;
        assert_eq!(
            validate_joint_allocation(&input),
            Err(JointAllocationError::MappingEvidenceStale)
        );
    }

    #[test]
    fn per_request_budget_is_checked_not_only_portfolio_cost() {
        let mut input = fixture();
        input.requests[0].max_total_cost_minor = 99;
        assert_eq!(
            validate_joint_allocation(&input),
            Err(JointAllocationError::RequestBudgetExceeded)
        );
    }

    #[test]
    fn duplicate_request_offer_lines_are_rejected_to_preserve_canonical_form() {
        let mut input = fixture();
        input.assignments.push(input.assignments[0].clone());
        assert_eq!(
            validate_joint_allocation(&input),
            Err(JointAllocationError::DuplicateAssignment)
        );
    }

    #[test]
    fn unit_or_currency_mismatches_do_not_enter_the_shared_plan() {
        let mut input = fixture();
        input.offers[0].currency = "EUR".to_owned();
        assert_eq!(
            validate_joint_allocation(&input),
            Err(JointAllocationError::CurrencyMismatch)
        );
    }

    #[test]
    fn aggregate_order_lot_is_checked_after_assignments_are_joined() {
        let mut input = fixture();
        input.offers = vec![offer("offer-a", "pool-a", 4, 10)];
        input.offers[0].minimum_order_quantity = 2;
        input.offers[0].order_increment = 2;
        input.capacity_pools = vec![pool("pool-a", 4)];
        input.requests = vec![request("urgent", 1, 100), request("routine", 1, 100)];
        input.capability_mappings = vec![mapping("urgent", "offer-a"), mapping("routine", "offer-a")];
        input.assignments = vec![
            JointAssignment { request_id: "urgent".to_owned(), offer_id: "offer-a".to_owned(), quantity: 1 },
        ];
        assert_eq!(
            validate_joint_allocation(&input),
            Err(JointAllocationError::InvalidOrderLot)
        );
        input.assignments.push(JointAssignment {
            request_id: "routine".to_owned(),
            offer_id: "offer-a".to_owned(),
            quantity: 1,
        });
        assert!(validate_joint_allocation(&input).is_ok());
    }

    #[test]
    fn canonical_report_is_independent_of_input_vector_order() {
        let mut first = fixture();
        let mut second = fixture();
        second.requests.reverse();
        second.offers.reverse();
        second.capacity_pools.reverse();
        second.capability_mappings.reverse();
        second.assignments.reverse();
        assert_eq!(
            validate_joint_allocation(&first),
            validate_joint_allocation(&second)
        );
        first.assignments.reverse();
        assert_eq!(
            validate_joint_allocation(&first),
            validate_joint_allocation(&second)
        );
    }

    #[test]
    fn deadline_violation_is_rejected_even_with_spare_capacity() {
        let mut input = fixture();
        input.offers[0].arrives_at = 501;
        assert_eq!(
            validate_joint_allocation(&input),
            Err(JointAllocationError::DeadlineMiss)
        );
    }
}
