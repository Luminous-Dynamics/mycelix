//! Capability-first, evidence-gated sourcing decision reference.
//!
//! This module filters hard constraints before comparing eligible offers. It
//! returns a Pareto frontier over landed cost, arrival time, and a version-bound
//! risk measure. It intentionally does not collapse those objectives into a
//! universal weighted score or choose a winner when trade-offs remain.
//!
//! Authority ceiling: deterministic comparison only. This module does not
//! authenticate supplier evidence, prove physical inventory, call the web, create
//! purchase orders, authorize spending, or execute transactions. A future adapter
//! must supply independently qualified evidence and obtain separate authorization.

use std::collections::BTreeSet;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EvidenceStatus {
    Verified,
    Unknown,
    Stale,
    Contested,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SourcingRequest {
    /// Explicit evaluation time; no hidden wall-clock dependency.
    pub as_of: u64,
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
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupplierOffer {
    pub offer_id: String,
    pub supplier_id: String,
    pub capability_id: String,
    pub capability_profile_digest: String,
    pub unit: String,
    pub currency: String,
    pub available_quantity: u64,
    /// Integer minor units, multiplied by the requested quantity.
    pub unit_price_minor: u64,
    /// All fixed costs are expressed in the request's currency and minor units.
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
    CapabilityMismatch,
    CapabilityProfileMismatch,
    UnitMismatch,
    CurrencyMismatch,
    RiskModelMismatch,
    RiskValueOutOfRange,
    QuoteNotVerified,
    SupplierCredentialNotVerified,
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
    pub landed_cost_minor: u64,
    pub arrival_at: u64,
    pub lead_time_secs: u64,
    pub risk_bps: u16,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RejectedOffer {
    pub offer_id: String,
    pub supplier_id: String,
    pub reason: RejectionReason,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SourcingReport {
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
    if offer.risk_bps > 10_000 {
        return Err(RejectionReason::RiskValueOutOfRange);
    }
    if offer.quote_evidence_status != EvidenceStatus::Verified {
        return Err(RejectionReason::QuoteNotVerified);
    }
    if offer.supplier_credential_status != EvidenceStatus::Verified {
        return Err(RejectionReason::SupplierCredentialNotVerified);
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
        landed_cost_minor: total,
        arrival_at,
        lead_time_secs: offer.lead_time_secs,
        risk_bps: offer.risk_bps,
    })
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
                reason,
            }),
        }
    }

    eligible.sort_by(|left, right| left.offer_id.cmp(&right.offer_id));
    rejected.sort_by(|left, right| left.offer_id.cmp(&right.offer_id));

    let mut pareto_frontier = Vec::new();
    let mut dominated_offers = Vec::new();
    for (index, candidate) in eligible.iter().enumerate() {
        let is_dominated = eligible
            .iter()
            .enumerate()
            .any(|(other_index, other)| other_index != index && dominates(other, candidate));
        if is_dominated {
            dominated_offers.push(candidate.clone());
        } else {
            pareto_frontier.push(candidate.clone());
        }
    }

    Ok(SourcingReport {
        pareto_frontier,
        dominated: dominated_offers,
        rejected,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request() -> SourcingRequest {
        SourcingRequest {
            as_of: 100,
            request_id: "need-1".into(),
            capability_id: "solar-charge-control".into(),
            capability_profile_digest: "profile-sha256-v1".into(),
            unit: "each".into(),
            currency: "USD".into(),
            quantity: 1,
            max_landed_cost_minor: 1_000,
            deliver_by: 500,
            max_quote_age_secs: 60,
            risk_model_digest: "risk-model-v1".into(),
        }
    }

    fn offer(id: &str, cost: u64, lead: u64, risk: u16) -> SupplierOffer {
        SupplierOffer {
            offer_id: id.into(),
            supplier_id: format!("supplier-{id}"),
            capability_id: "solar-charge-control".into(),
            capability_profile_digest: "profile-sha256-v1".into(),
            unit: "each".into(),
            currency: "USD".into(),
            available_quantity: 100,
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
        assert_eq!(report.rejected[0].reason, RejectionReason::CapabilityProfileMismatch);
        assert_eq!(report.rejected[1].reason, RejectionReason::QuoteNotVerified);
        assert_eq!(report.rejected[2].reason, RejectionReason::QuoteStale);
        assert_eq!(report.rejected[3].reason, RejectionReason::SupplierCredentialExpired);
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
        assert_eq!(report.rejected[0].reason, RejectionReason::LandedCostOverflow);
    }

    #[test]
    fn mismatched_currency_and_risk_models_are_not_compared() {
        let mut foreign_currency = offer("eur", 10, 1, 1);
        foreign_currency.currency = "EUR".into();
        let mut other_risk_model = offer("risk-v2", 10, 1, 1);
        other_risk_model.risk_model_digest = "risk-model-v2".into();
        let report =
            evaluate_sourcing(&request(), &[foreign_currency, other_risk_model]).unwrap();
        assert!(report.pareto_frontier.is_empty());
        assert_eq!(report.rejected.len(), 2);
        assert_eq!(report.rejected[0].reason, RejectionReason::CurrencyMismatch);
        assert_eq!(report.rejected[1].reason, RejectionReason::RiskModelMismatch);
    }

    #[test]
    fn equal_objectives_do_not_create_an_arbitrary_winner() {
        let report = evaluate_sourcing(
            &request(),
            &[offer("offer-a", 100, 10, 100), offer("offer-b", 100, 10, 100)],
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
        assert_eq!(report.rejected[0].reason, RejectionReason::InsufficientQuantity);
    }

    #[test]
    fn late_delivery_is_rejected_even_when_cheap() {
        let candidate = offer("late", 1, 401, 1);
        let report = evaluate_sourcing(&request(), &[candidate]).unwrap();
        assert_eq!(report.rejected[0].reason, RejectionReason::CannotMeetDeadline);
        assert!(report.pareto_frontier.is_empty());
    }
}
