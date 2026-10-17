//! Capability-first, evidence-gated sourcing decision reference.
//!
//! This module filters hard constraints before comparing eligible offers. It
//! returns a Pareto frontier over landed cost, arrival time, and a version-bound
//! risk measure. It intentionally does not collapse those objectives into a
//! universal weighted score or choose a winner when trade-offs remain.
//!
//! Authority ceiling: deterministic comparison only. This module does not
//! authenticate supplier evidence, prove physical inventory, call the web, create
//! purchase orders, authorize spending, or execute transactions.
//!
//! The public structs are evaluator input projections, not canonical supplier-listing
//! or physical-component identities. Compose with upstream component/substitution,
//! EPI provenance, and supply-chain contracts rather than treating this crate as a
//! second product or supplier master-data registry. A future adapter
//! must supply independently qualified evidence and obtain separate authorization.

use std::collections::BTreeSet;

// Reuse the crate-wide evidence semantics; do not mint a sourcing-specific duplicate.
pub use super::{EvidenceOrigin, EvidenceStatus};

/// Versioned identity for the deterministic sourcing-comparison algorithm.
pub const SOURCING_EVALUATOR_PROFILE: &str = "mycelix:sourcing-pareto-fenwick:v1";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SourcingRequest {
    /// Explicit evaluation time; no hidden wall-clock dependency.
    pub as_of: u64,
    /// Prevents live evidence and synthetic simulation records from being mixed.
    pub required_evidence_origin: EvidenceOrigin,
    pub request_id: String,
    /// Canonical capability identifier, not a retailer-specific SKU.
    pub capability_id: String,
    /// Digest of the versioned profile that defines mandatory capability semantics.
    pub capability_profile_digest: String,
    /// Canonical unit identifier; no implicit unit conversion is performed.
    pub unit: String,
    /// Canonical currency identifier; no implicit FX conversion is performed.
    pub currency: String,
    pub quantity: u64,
    pub max_landed_cost_minor: u64,
    pub deliver_by: u64,
    pub max_quote_age_secs: u64,
    pub max_inventory_age_secs: u64,
    pub max_credential_status_age_secs: u64,
    /// Binds the meaning and calibration of risk_bps across candidates.
    pub risk_model_digest: String,
}

impl SourcingRequest {
    fn is_valid(&self) -> bool {
        [
            self.request_id.as_str(),
            self.capability_id.as_str(),
            self.capability_profile_digest.as_str(),
            self.unit.as_str(),
            self.currency.as_str(),
            self.risk_model_digest.as_str(),
        ]
        .iter()
        .all(|value| !value.trim().is_empty())
            && self.quantity > 0
            && self.deliver_by >= self.as_of
            && self.required_evidence_origin != EvidenceOrigin::Unknown
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
/// Minimal offer projection for deterministic comparison, not a canonical listing record.
pub struct SupplierOffer {
    pub offer_id: String,
    pub supplier_id: String,
    /// Source record locator captured by the discovery adapter.
    pub source_uri: String,
    /// Opaque digest reference for exact source payload bytes; content is not verified here.
    pub source_payload_digest: String,
    /// Version/digest identifying the extraction profile that produced the fields.
    pub extractor_profile_id: String,
    pub quote_evidence_origin: EvidenceOrigin,
    pub supplier_credential_origin: EvidenceOrigin,
    pub capability_id: String,
    pub capability_profile_digest: String,
    pub unit: String,
    pub currency: String,
    pub available_quantity: u64,
    /// Exact time at which the stock observation was made.
    pub inventory_observed_at: u64,
    pub inventory_valid_until: u64,
    pub inventory_evidence_status: EvidenceStatus,
    pub inventory_evidence_origin: EvidenceOrigin,
    /// Integer minor units, multiplied by the requested quantity.
    pub unit_price_minor: u64,
    /// Order-level totals in the request currency/minor unit for the requested quantity.
    /// Adapters must normalize per-unit fees into these fields before evaluation.
    pub shipping_minor: u64,
    pub taxes_minor: u64,
    pub other_fees_minor: u64,
    pub lead_time_secs: u64,
    /// Normalized 0..=10,000 value under risk_model_digest; not a universal score.
    pub risk_bps: u16,
    pub risk_model_digest: String,
    pub quote_observed_at: u64,
    pub quote_valid_until: u64,
    pub quote_evidence_status: EvidenceStatus,
    pub supplier_credential_status: EvidenceStatus,
    /// Time of the last credential validity/revocation/status check.
    pub supplier_credential_checked_at: u64,
    pub supplier_credential_valid_until: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EvaluationError {
    InvalidRequest,
    DuplicateOfferId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RejectionReason {
    InvalidOfferIdentity,
    MissingSourceProvenance,
    QuoteEvidenceOriginMismatch,
    SupplierCredentialOriginMismatch,
    InventoryOriginMismatch,
    InventoryNotVerified,
    InventoryFromFuture,
    InventoryStale,
    InventoryExpired,
    CapabilityMismatch,
    CapabilityProfileMismatch,
    UnitMismatch,
    CurrencyMismatch,
    RiskModelMismatch,
    RiskValueOutOfRange,
    QuoteNotVerified,
    SupplierCredentialNotVerified,
    SupplierCredentialCheckFromFuture,
    SupplierCredentialStatusStale,
    QuoteFromFuture,
    QuoteStale,
    QuoteExpired,
    SupplierCredentialExpired,
    InsufficientQuantity,
    ArrivalTimeOverflow,
    CannotMeetDeadline,
    LandedCostOverflow,
    OverBudget,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvaluatedOffer {
    pub offer_id: String,
    pub supplier_id: String,
    pub source_uri: String,
    pub source_payload_digest: String,
    pub extractor_profile_id: String,
    pub quote_evidence_origin: EvidenceOrigin,
    pub quote_observed_at: u64,
    pub quote_valid_until: u64,
    pub quote_evidence_status: EvidenceStatus,
    pub inventory_observed_at: u64,
    pub inventory_valid_until: u64,
    pub inventory_evidence_status: EvidenceStatus,
    pub inventory_evidence_origin: EvidenceOrigin,
    pub supplier_credential_origin: EvidenceOrigin,
    pub supplier_credential_status: EvidenceStatus,
    /// Time of the last credential validity/revocation/status check.
    pub supplier_credential_checked_at: u64,
    pub supplier_credential_valid_until: u64,
    pub landed_cost_minor: u64,
    pub arrival_at: u64,
    pub lead_time_secs: u64,
    pub risk_bps: u16,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RejectedOffer {
    pub offer_id: String,
    pub supplier_id: String,
    pub source_uri: String,
    pub source_payload_digest: String,
    pub extractor_profile_id: String,
    pub quote_evidence_origin: EvidenceOrigin,
    pub quote_observed_at: u64,
    pub quote_valid_until: u64,
    pub quote_evidence_status: EvidenceStatus,
    pub inventory_observed_at: u64,
    pub inventory_valid_until: u64,
    pub inventory_evidence_status: EvidenceStatus,
    pub inventory_evidence_origin: EvidenceOrigin,
    pub supplier_credential_origin: EvidenceOrigin,
    pub supplier_credential_status: EvidenceStatus,
    /// Time of the last credential validity/revocation/status check.
    pub supplier_credential_checked_at: u64,
    pub supplier_credential_valid_until: u64,
    pub reason: RejectionReason,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SourcingReport {
    /// Exact comparison algorithm/profile identity used for this report.
    pub evaluator_profile: &'static str,
    /// Immutable request/evaluation context needed to interpret and replay this report.
    pub request_id: String,
    pub evaluated_at: u64,
    pub required_evidence_origin: EvidenceOrigin,
    pub capability_id: String,
    pub capability_profile_digest: String,
    pub unit: String,
    pub currency: String,
    pub quantity: u64,
    pub risk_model_digest: String,
    /// All eligible offers that are not dominated on every declared objective.
    /// Multiple frontier members mean the system has found a trade-off, not a winner.
    pub pareto_frontier: Vec<EvaluatedOffer>,
    /// Offers for which at least one other eligible offer is no worse on all
    /// objectives and strictly better on at least one.
    pub dominated: Vec<EvaluatedOffer>,
    /// Ineligible offers, with one deterministic first-failure reason each.
    pub rejected: Vec<RejectedOffer>,
}

impl SourcingReport {
    pub fn eligible_count(&self) -> usize {
        self.pareto_frontier.len() + self.dominated.len()
    }
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn landed_cost(request: &SourcingRequest, offer: &SupplierOffer) -> Result<u64, RejectionReason> {
    offer
        .unit_price_minor
        .checked_mul(request.quantity)
        .and_then(|amount| amount.checked_add(offer.shipping_minor))
        .and_then(|amount| amount.checked_add(offer.taxes_minor))
        .and_then(|amount| amount.checked_add(offer.other_fees_minor))
        .ok_or(RejectionReason::LandedCostOverflow)
}

fn evaluate_offer(
    request: &SourcingRequest,
    offer: &SupplierOffer,
) -> Result<EvaluatedOffer, RejectionReason> {
    if !non_empty(&offer.offer_id) || !non_empty(&offer.supplier_id) {
        return Err(RejectionReason::InvalidOfferIdentity);
    }
    if !non_empty(&offer.source_uri)
        || !non_empty(&offer.source_payload_digest)
        || !non_empty(&offer.extractor_profile_id)
    {
        return Err(RejectionReason::MissingSourceProvenance);
    }
    if offer.capability_id != request.capability_id {
        return Err(RejectionReason::CapabilityMismatch);
    }
    if offer.capability_profile_digest != request.capability_profile_digest {
        return Err(RejectionReason::CapabilityProfileMismatch);
    }
    if offer.unit != request.unit {
        return Err(RejectionReason::UnitMismatch);
    }
    if offer.currency != request.currency {
        return Err(RejectionReason::CurrencyMismatch);
    }
    if offer.risk_model_digest != request.risk_model_digest {
        return Err(RejectionReason::RiskModelMismatch);
    }
    if offer.quote_evidence_origin != request.required_evidence_origin {
        return Err(RejectionReason::QuoteEvidenceOriginMismatch);
    }
    if offer.supplier_credential_origin != request.required_evidence_origin {
        return Err(RejectionReason::SupplierCredentialOriginMismatch);
    }
    if offer.risk_bps > 10_000 {
        return Err(RejectionReason::RiskValueOutOfRange);
    }
    if offer.quote_evidence_status != EvidenceStatus::Verified {
        return Err(RejectionReason::QuoteNotVerified);
    }
    if offer.inventory_evidence_origin != request.required_evidence_origin {
        return Err(RejectionReason::InventoryOriginMismatch);
    }
    if offer.inventory_evidence_status != EvidenceStatus::Verified {
        return Err(RejectionReason::InventoryNotVerified);
    }
    if offer.inventory_observed_at > request.as_of {
        return Err(RejectionReason::InventoryFromFuture);
    }
    if request.as_of - offer.inventory_observed_at > request.max_inventory_age_secs {
        return Err(RejectionReason::InventoryStale);
    }
    if offer.inventory_valid_until < request.as_of {
        return Err(RejectionReason::InventoryExpired);
    }
    if offer.supplier_credential_status != EvidenceStatus::Verified {
        return Err(RejectionReason::SupplierCredentialNotVerified);
    }
    if offer.supplier_credential_checked_at > request.as_of {
        return Err(RejectionReason::SupplierCredentialCheckFromFuture);
    }
    if request.as_of - offer.supplier_credential_checked_at > request.max_credential_status_age_secs
    {
        return Err(RejectionReason::SupplierCredentialStatusStale);
    }
    if offer.quote_observed_at > request.as_of {
        return Err(RejectionReason::QuoteFromFuture);
    }
    if request.as_of - offer.quote_observed_at > request.max_quote_age_secs {
        return Err(RejectionReason::QuoteStale);
    }
    if offer.quote_valid_until < request.as_of {
        return Err(RejectionReason::QuoteExpired);
    }
    if offer.supplier_credential_valid_until < request.as_of {
        return Err(RejectionReason::SupplierCredentialExpired);
    }
    if offer.available_quantity < request.quantity {
        return Err(RejectionReason::InsufficientQuantity);
    }

    let arrival_at = request
        .as_of
        .checked_add(offer.lead_time_secs)
        .ok_or(RejectionReason::ArrivalTimeOverflow)?;
    if arrival_at > request.deliver_by {
        return Err(RejectionReason::CannotMeetDeadline);
    }

    let total = landed_cost(request, offer)?;
    if total > request.max_landed_cost_minor {
        return Err(RejectionReason::OverBudget);
    }

    Ok(EvaluatedOffer {
        offer_id: offer.offer_id.clone(),
        supplier_id: offer.supplier_id.clone(),
        source_uri: offer.source_uri.clone(),
        source_payload_digest: offer.source_payload_digest.clone(),
        extractor_profile_id: offer.extractor_profile_id.clone(),
        quote_evidence_origin: offer.quote_evidence_origin,
        quote_observed_at: offer.quote_observed_at,
        quote_valid_until: offer.quote_valid_until,
        quote_evidence_status: offer.quote_evidence_status,
        inventory_observed_at: offer.inventory_observed_at,
        inventory_valid_until: offer.inventory_valid_until,
        inventory_evidence_status: offer.inventory_evidence_status,
        inventory_evidence_origin: offer.inventory_evidence_origin,
        supplier_credential_origin: offer.supplier_credential_origin,
        supplier_credential_status: offer.supplier_credential_status,
        supplier_credential_checked_at: offer.supplier_credential_checked_at,
        supplier_credential_valid_until: offer.supplier_credential_valid_until,
        landed_cost_minor: total,
        arrival_at,
        lead_time_secs: offer.lead_time_secs,
        risk_bps: offer.risk_bps,
    })
}

/// Prefix-min Fenwick tree over compressed arrival-time coordinates.
/// Each node retains the lowest risk value observed in its prefix. Combined
/// with the cost/arrival/risk sweep below, this answers whether any already-seen
/// offer dominates the current objective point in O(log n).
struct PrefixMinFenwick {
    tree: Vec<Option<u16>>,
}

impl PrefixMinFenwick {
    fn new(coordinate_count: usize) -> Self {
        Self {
            tree: vec![None; coordinate_count + 1],
        }
    }

    fn update(&mut self, coordinate_index: usize, value: u16) {
        let mut index = coordinate_index + 1;
        while index < self.tree.len() {
            self.tree[index] = Some(match self.tree[index] {
                Some(previous) => previous.min(value),
                None => value,
            });
            index += index & (!index + 1);
        }
    }

    fn query_prefix_min(&self, coordinate_index: usize) -> Option<u16> {
        let mut index = coordinate_index + 1;
        let mut minimum: Option<u16> = None;
        while index > 0 {
            minimum = match (minimum, self.tree[index]) {
                (Some(previous), Some(value)) => Some(previous.min(value)),
                (Some(previous), None) => Some(previous),
                (None, Some(value)) => Some(value),
                (None, None) => None,
            };
            index &= index - 1;
        }
        minimum
    }
}

/// Classifies a minimization frontier in O(n log n) after sorting.
///
/// Sweep order is landed cost, arrival, then risk, all ascending. For the
/// current objective point, the Fenwick prefix minimum answers whether any
/// previously processed point has arrival <= the current arrival and risk <=
/// the current risk. Such a point also has cost <= current cost due to sweep
/// order. Exact objective duplicates are processed as a group before updating
/// the tree, so duplicate suppliers with identical objectives do not dominate
/// one another arbitrarily.
fn classify_frontier(
    mut eligible: Vec<EvaluatedOffer>,
) -> (Vec<EvaluatedOffer>, Vec<EvaluatedOffer>) {
    if eligible.len() < 2 {
        return (eligible, Vec::new());
    }

    let mut arrival_coordinates: Vec<u64> = eligible.iter().map(|offer| offer.arrival_at).collect();
    arrival_coordinates.sort_unstable();
    arrival_coordinates.dedup();

    eligible.sort_by(|left, right| {
        left.landed_cost_minor
            .cmp(&right.landed_cost_minor)
            .then_with(|| left.arrival_at.cmp(&right.arrival_at))
            .then_with(|| left.risk_bps.cmp(&right.risk_bps))
            .then_with(|| left.offer_id.cmp(&right.offer_id))
    });

    let mut index_by_arrival = Vec::with_capacity(eligible.len());
    for offer in &eligible {
        // The value was collected from this same eligible set.
        let index = arrival_coordinates
            .binary_search(&offer.arrival_at)
            .expect("every eligible arrival has a compressed coordinate");
        index_by_arrival.push(index);
    }

    let mut tree = PrefixMinFenwick::new(arrival_coordinates.len());
    let mut frontier = Vec::new();
    let mut dominated = Vec::new();
    let mut start = 0;

    while start < eligible.len() {
        let first = &eligible[start];
        let mut end = start + 1;
        while end < eligible.len()
            && eligible[end].landed_cost_minor == first.landed_cost_minor
            && eligible[end].arrival_at == first.arrival_at
            && eligible[end].risk_bps == first.risk_bps
        {
            end += 1;
        }

        let arrival_index = index_by_arrival[start];
        let is_dominated = tree
            .query_prefix_min(arrival_index)
            .is_some_and(|minimum_risk| minimum_risk <= first.risk_bps);

        if is_dominated {
            dominated.extend(eligible[start..end].iter().cloned());
        } else {
            frontier.extend(eligible[start..end].iter().cloned());
        }

        // Insert the whole exact-objective group only after classification.
        for index in start..end {
            tree.update(index_by_arrival[index], eligible[index].risk_bps);
        }
        start = end;
    }

    frontier.sort_by(|left, right| left.offer_id.cmp(&right.offer_id));
    dominated.sort_by(|left, right| left.offer_id.cmp(&right.offer_id));
    (frontier, dominated)
}

fn dominates(left: &EvaluatedOffer, right: &EvaluatedOffer) -> bool {
    let no_worse = left.landed_cost_minor <= right.landed_cost_minor
        && left.arrival_at <= right.arrival_at
        && left.risk_bps <= right.risk_bps;
    let strictly_better = left.landed_cost_minor < right.landed_cost_minor
        || left.arrival_at < right.arrival_at
        || left.risk_bps < right.risk_bps;
    no_worse && strictly_better
}

/// Evaluates supplier offers at a fixed time, filters hard constraints, and
/// returns the non-dominated set. Results are sorted by offer ID for replayability.
///
/// Exact capability-profile and risk-model digest matching is deliberate. An
/// LLM or web scraper may discover candidate offers, but must not silently
/// declare a substitute equivalent or compare incompatible risk measurements.
pub fn evaluate_sourcing(
    request: &SourcingRequest,
    offers: &[SupplierOffer],
) -> Result<SourcingReport, EvaluationError> {
    if !request.is_valid() {
        return Err(EvaluationError::InvalidRequest);
    }

    let mut seen_ids = BTreeSet::new();
    for offer in offers {
        if !offer.offer_id.trim().is_empty() && !seen_ids.insert(offer.offer_id.clone()) {
            return Err(EvaluationError::DuplicateOfferId);
        }
    }

    let mut eligible = Vec::new();
    let mut rejected = Vec::new();
    for offer in offers {
        match evaluate_offer(request, offer) {
            Ok(evaluated) => eligible.push(evaluated),
            Err(reason) => rejected.push(RejectedOffer {
                offer_id: offer.offer_id.clone(),
                supplier_id: offer.supplier_id.clone(),
                source_uri: offer.source_uri.clone(),
                source_payload_digest: offer.source_payload_digest.clone(),
                extractor_profile_id: offer.extractor_profile_id.clone(),
                quote_evidence_origin: offer.quote_evidence_origin,
                quote_observed_at: offer.quote_observed_at,
                quote_valid_until: offer.quote_valid_until,
                quote_evidence_status: offer.quote_evidence_status,
                inventory_observed_at: offer.inventory_observed_at,
                inventory_valid_until: offer.inventory_valid_until,
                inventory_evidence_status: offer.inventory_evidence_status,
                inventory_evidence_origin: offer.inventory_evidence_origin,
                supplier_credential_origin: offer.supplier_credential_origin,
                supplier_credential_status: offer.supplier_credential_status,
                supplier_credential_checked_at: offer.supplier_credential_checked_at,
                supplier_credential_valid_until: offer.supplier_credential_valid_until,
                reason,
            }),
        }
    }

    rejected.sort_by(|left, right| left.offer_id.cmp(&right.offer_id));

    let (pareto_frontier, dominated) = classify_frontier(eligible);

    Ok(SourcingReport {
        evaluator_profile: SOURCING_EVALUATOR_PROFILE,
        request_id: request.request_id.clone(),
        evaluated_at: request.as_of,
        required_evidence_origin: request.required_evidence_origin,
        capability_id: request.capability_id.clone(),
        capability_profile_digest: request.capability_profile_digest.clone(),
        unit: request.unit.clone(),
        currency: request.currency.clone(),
        quantity: request.quantity,
        risk_model_digest: request.risk_model_digest.clone(),
        pareto_frontier,
        dominated,
        rejected,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request() -> SourcingRequest {
        SourcingRequest {
            as_of: 100,
            required_evidence_origin: EvidenceOrigin::SyntheticSimulation,
            request_id: "need-1".into(),
            capability_id: "solar-charge-control".into(),
            capability_profile_digest: "profile-sha256-v1".into(),
            unit: "each".into(),
            currency: "USD".into(),
            quantity: 1,
            max_landed_cost_minor: 1_000,
            deliver_by: 500,
            max_quote_age_secs: 60,
            max_inventory_age_secs: 60,
            max_credential_status_age_secs: 60,
            risk_model_digest: "risk-model-v1".into(),
        }
    }

    fn offer(id: &str, cost: u64, lead: u64, risk: u16) -> SupplierOffer {
        SupplierOffer {
            offer_id: id.into(),
            supplier_id: format!("supplier-{id}"),
            source_uri: format!("https://supplier.example/offers/{id}"),
            source_payload_digest: format!("fixture-source-digest-{id}"),
            extractor_profile_id: "fixture-extractor-v1".into(),
            quote_evidence_origin: EvidenceOrigin::SyntheticSimulation,
            supplier_credential_origin: EvidenceOrigin::SyntheticSimulation,
            capability_id: "solar-charge-control".into(),
            capability_profile_digest: "profile-sha256-v1".into(),
            unit: "each".into(),
            currency: "USD".into(),
            available_quantity: 100,
            inventory_observed_at: 100,
            inventory_valid_until: 200,
            inventory_evidence_status: EvidenceStatus::Verified,
            inventory_evidence_origin: EvidenceOrigin::SyntheticSimulation,
            unit_price_minor: cost,
            shipping_minor: 0,
            taxes_minor: 0,
            other_fees_minor: 0,
            lead_time_secs: lead,
            risk_bps: risk,
            risk_model_digest: "risk-model-v1".into(),
            quote_observed_at: 100,
            quote_valid_until: 200,
            quote_evidence_status: EvidenceStatus::Verified,
            supplier_credential_status: EvidenceStatus::Verified,
            supplier_credential_checked_at: 100,
            supplier_credential_valid_until: 500,
        }
    }

    #[test]
    fn pareto_frontier_preserves_cost_speed_risk_tradeoffs() {
        let offers = vec![
            offer("a-balanced", 100, 10, 1_000),
            offer("b-cheap", 80, 15, 5_000),
            offer("c-dominated", 150, 30, 9_000),
        ];
        let report = evaluate_sourcing(&request(), &offers).unwrap();
        assert_eq!(
            report
                .pareto_frontier
                .iter()
                .map(|item| item.offer_id.as_str())
                .collect::<Vec<_>>(),
            vec!["a-balanced", "b-cheap"]
        );
        assert_eq!(report.dominated.len(), 1);
        assert_eq!(report.dominated[0].offer_id, "c-dominated");
    }

    #[test]
    fn hard_constraints_filter_before_pareto_comparison() {
        let mut incompatible = offer("a-cheapest", 1, 1, 1);
        incompatible.capability_profile_digest = "different-profile".into();
        let mut unverified = offer("b-unverified", 2, 1, 1);
        unverified.quote_evidence_status = EvidenceStatus::Unknown;
        let mut stale = offer("c-stale", 3, 1, 1);
        stale.quote_observed_at = 39;
        let mut expired_credential = offer("d-expired", 4, 1, 1);
        expired_credential.supplier_credential_valid_until = 99;

        let report = evaluate_sourcing(
            &request(),
            &[
                incompatible,
                unverified,
                stale,
                expired_credential,
                offer("e-eligible", 100, 10, 1_000),
            ],
        )
        .unwrap();

        assert_eq!(report.pareto_frontier.len(), 1);
        assert_eq!(report.pareto_frontier[0].offer_id, "e-eligible");
        assert_eq!(report.rejected.len(), 4);
        assert_eq!(
            report.rejected[0].reason,
            RejectionReason::CapabilityProfileMismatch
        );
        assert_eq!(report.rejected[1].reason, RejectionReason::QuoteNotVerified);
        assert_eq!(report.rejected[2].reason, RejectionReason::QuoteStale);
        assert_eq!(
            report.rejected[3].reason,
            RejectionReason::SupplierCredentialExpired
        );
    }

    #[test]
    fn exact_freshness_boundary_is_accepted() {
        let mut candidate = offer("at-boundary", 100, 10, 100);
        candidate.quote_observed_at = 40;
        let report = evaluate_sourcing(&request(), &[candidate]).unwrap();
        assert_eq!(report.eligible_count(), 1);
        assert!(report.rejected.is_empty());
    }

    #[test]
    fn cost_overflow_is_rejected_not_wrapped() {
        let mut req = request();
        req.quantity = 2;
        req.max_landed_cost_minor = u64::MAX;
        let mut candidate = offer("overflow", u64::MAX, 1, 0);
        candidate.available_quantity = 2;
        let report = evaluate_sourcing(&req, &[candidate]).unwrap();
        assert_eq!(report.rejected.len(), 1);
        assert_eq!(
            report.rejected[0].reason,
            RejectionReason::LandedCostOverflow
        );
    }

    #[test]
    fn mismatched_currency_and_risk_models_are_not_compared() {
        let mut foreign_currency = offer("eur", 10, 1, 1);
        foreign_currency.currency = "EUR".into();
        let mut other_risk_model = offer("risk-v2", 10, 1, 1);
        other_risk_model.risk_model_digest = "risk-model-v2".into();
        let report = evaluate_sourcing(&request(), &[foreign_currency, other_risk_model]).unwrap();
        assert!(report.pareto_frontier.is_empty());
        assert_eq!(report.rejected.len(), 2);
        assert_eq!(report.rejected[0].reason, RejectionReason::CurrencyMismatch);
        assert_eq!(
            report.rejected[1].reason,
            RejectionReason::RiskModelMismatch
        );
    }

    #[test]
    fn equal_objectives_do_not_create_an_arbitrary_winner() {
        let report = evaluate_sourcing(
            &request(),
            &[
                offer("offer-a", 100, 10, 100),
                offer("offer-b", 100, 10, 100),
            ],
        )
        .unwrap();
        assert_eq!(report.pareto_frontier.len(), 2);
        assert!(report.dominated.is_empty());
    }

    #[test]
    fn duplicate_offer_ids_fail_closed() {
        let duplicate = vec![offer("same", 100, 10, 100), offer("same", 90, 8, 90)];
        assert_eq!(
            evaluate_sourcing(&request(), &duplicate),
            Err(EvaluationError::DuplicateOfferId)
        );
    }

    #[test]
    fn output_is_invariant_to_input_order() {
        let first = vec![
            offer("b", 80, 15, 5_000),
            offer("a", 100, 10, 1_000),
            offer("c", 150, 30, 9_000),
        ];
        let second = vec![first[2].clone(), first[0].clone(), first[1].clone()];
        assert_eq!(
            evaluate_sourcing(&request(), &first).unwrap(),
            evaluate_sourcing(&request(), &second).unwrap()
        );
    }

    #[test]
    fn no_eligible_offer_means_empty_frontier_not_forced_selection() {
        let mut candidate = offer("not-enough-stock", 10, 1, 1);
        candidate.available_quantity = 0;
        let report = evaluate_sourcing(&request(), &[candidate]).unwrap();
        assert!(report.pareto_frontier.is_empty());
        assert!(report.dominated.is_empty());
        assert_eq!(
            report.rejected[0].reason,
            RejectionReason::InsufficientQuantity
        );
    }

    #[test]
    fn missing_source_provenance_is_rejected_and_reported() {
        let mut candidate = offer("no-source-digest", 100, 10, 100);
        candidate.source_payload_digest.clear();
        let report = evaluate_sourcing(&request(), &[candidate]).unwrap();
        assert_eq!(
            report.rejected[0].reason,
            RejectionReason::MissingSourceProvenance
        );
        assert_eq!(
            report.rejected[0].source_uri,
            "https://supplier.example/offers/no-source-digest"
        );
        assert!(report.pareto_frontier.is_empty());
    }

    #[test]
    fn future_or_expired_quotes_are_rejected() {
        let mut future = offer("future", 100, 10, 100);
        future.quote_observed_at = 101;
        let mut expired = offer("expired", 90, 10, 100);
        expired.quote_valid_until = 99;
        let report = evaluate_sourcing(&request(), &[future, expired]).unwrap();
        assert_eq!(report.rejected[0].reason, RejectionReason::QuoteExpired);
        assert_eq!(report.rejected[1].reason, RejectionReason::QuoteFromFuture);
        assert!(report.pareto_frontier.is_empty());
    }

    #[test]
    fn over_budget_and_noncanonical_units_are_rejected() {
        let mut req = request();
        req.max_landed_cost_minor = 99;
        let cheap_but_wrong_unit = {
            let mut item = offer("wrong-unit", 1, 1, 1);
            item.unit = "case".into();
            item
        };
        let over_budget = offer("over-budget", 100, 1, 1);
        let report = evaluate_sourcing(&req, &[cheap_but_wrong_unit, over_budget]).unwrap();
        assert_eq!(report.rejected[0].reason, RejectionReason::OverBudget);
        assert_eq!(report.rejected[1].reason, RejectionReason::UnitMismatch);
        assert!(report.pareto_frontier.is_empty());
    }

    #[test]
    fn invalid_risk_value_and_arrival_overflow_fail_closed() {
        let mut invalid_risk = offer("invalid-risk", 100, 1, 10_001);
        invalid_risk.risk_model_digest = "risk-model-v1".into();

        let mut req = request();
        req.as_of = u64::MAX - 5;
        req.deliver_by = u64::MAX;
        req.max_quote_age_secs = 100;
        let mut overflow_arrival = offer("arrival-overflow", 100, 10, 100);
        overflow_arrival.quote_observed_at = req.as_of;
        overflow_arrival.quote_valid_until = u64::MAX;
        overflow_arrival.inventory_observed_at = req.as_of;
        overflow_arrival.inventory_valid_until = u64::MAX;
        overflow_arrival.supplier_credential_checked_at = req.as_of;
        overflow_arrival.supplier_credential_valid_until = u64::MAX;
        let report = evaluate_sourcing(&req, &[overflow_arrival]).unwrap();
        assert_eq!(report.rejected[0].reason, RejectionReason::ArrivalTimeOverflow);

        let report = evaluate_sourcing(&request(), &[invalid_risk]).unwrap();
        assert_eq!(report.rejected[0].reason, RejectionReason::RiskValueOutOfRange);
    }

    #[test]
    fn live_and_synthetic_evidence_origins_cannot_be_mixed() {
        let mut candidate = offer("live-in-simulation", 100, 10, 100);
        candidate.quote_evidence_origin = EvidenceOrigin::LiveDomain;
        let report = evaluate_sourcing(&request(), &[candidate]).unwrap();
        assert_eq!(
            report.rejected[0].reason,
            RejectionReason::QuoteEvidenceOriginMismatch
        );
        assert!(report.pareto_frontier.is_empty());
    }

    #[test]
    fn fresh_quote_does_not_rescue_stale_or_unverified_inventory() {
        let mut stale_stock = offer("stale-stock", 10, 1, 10);
        stale_stock.inventory_observed_at = 39;
        let mut unverified_stock = offer("unverified-stock", 10, 1, 10);
        unverified_stock.inventory_evidence_status = EvidenceStatus::Unknown;
        let mut expired_stock = offer("expired-stock", 10, 1, 10);
        expired_stock.inventory_valid_until = 99;

        let report =
            evaluate_sourcing(&request(), &[stale_stock, unverified_stock, expired_stock]).unwrap();
        assert_eq!(report.rejected[0].reason, RejectionReason::InventoryExpired);
        assert_eq!(report.rejected[1].reason, RejectionReason::InventoryStale);
        assert_eq!(
            report.rejected[2].reason,
            RejectionReason::InventoryNotVerified
        );
        assert!(report.pareto_frontier.is_empty());
    }

    #[test]
    fn inventory_origin_must_match_the_request_origin() {
        let mut candidate = offer("inventory-live-in-simulation", 10, 1, 10);
        candidate.inventory_evidence_origin = EvidenceOrigin::LiveDomain;
        let report = evaluate_sourcing(&request(), &[candidate]).unwrap();
        assert_eq!(
            report.rejected[0].reason,
            RejectionReason::InventoryOriginMismatch
        );
        assert!(report.pareto_frontier.is_empty());
    }

    #[test]
    fn report_preserves_the_exact_quote_and_inventory_observation_times() {
        let mut candidate = offer("timestamp-preserved", 100, 10, 100);
        candidate.quote_observed_at = 80;
        candidate.quote_valid_until = 140;
        candidate.inventory_observed_at = 90;
        candidate.inventory_valid_until = 150;

        let report = evaluate_sourcing(&request(), &[candidate]).unwrap();
        let selected = &report.pareto_frontier[0];
        assert_eq!(selected.quote_observed_at, 80);
        assert_eq!(selected.quote_valid_until, 140);
        assert_eq!(selected.inventory_observed_at, 90);
        assert_eq!(selected.inventory_valid_until, 150);
    }

    #[test]
    fn report_binds_request_and_objective_model_context() {
        let report = evaluate_sourcing(&request(), &[offer("context", 100, 10, 100)]).unwrap();
        assert_eq!(
            report.evaluator_profile,
            "mycelix:sourcing-pareto-fenwick:v1"
        );
        assert_eq!(report.request_id, "need-1");
        assert_eq!(report.evaluated_at, 100);
        assert_eq!(
            report.required_evidence_origin,
            EvidenceOrigin::SyntheticSimulation
        );
        assert_eq!(report.capability_id, "solar-charge-control");
        assert_eq!(report.capability_profile_digest, "profile-sha256-v1");
        assert_eq!(report.unit, "each");
        assert_eq!(report.currency, "USD");
        assert_eq!(report.quantity, 1);
        assert_eq!(report.risk_model_digest, "risk-model-v1");
    }

    #[test]
    fn credential_expiry_does_not_rescue_stale_status_check() {
        let mut stale = offer("stale-credential-status", 100, 10, 100);
        stale.supplier_credential_checked_at = 39;
        let mut future = offer("future-credential-status", 100, 10, 100);
        future.supplier_credential_checked_at = 101;

        let report = evaluate_sourcing(&request(), &[stale, future]).unwrap();
        assert_eq!(
            report.rejected[0].reason,
            RejectionReason::SupplierCredentialCheckFromFuture
        );
        assert_eq!(
            report.rejected[1].reason,
            RejectionReason::SupplierCredentialStatusStale
        );
        assert!(report.pareto_frontier.is_empty());
    }

    #[test]
    fn late_delivery_is_rejected_even_when_cheap() {
        let candidate = offer("late", 1, 401, 1);
        let report = evaluate_sourcing(&request(), &[candidate]).unwrap();
        assert_eq!(
            report.rejected[0].reason,
            RejectionReason::CannotMeetDeadline
        );
        assert!(report.pareto_frontier.is_empty());
    }
}
