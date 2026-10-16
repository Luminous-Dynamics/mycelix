//! Bounded exact portfolio-allocation reference model.
//!
//! This is an effects-disabled optimizer for small, already-admitted offers. It
//! keeps hard eligibility gates separate from portfolio allocation and returns
//! every non-dominated feasible allocation it finds within explicit search bounds.
//! It is not a supplier-authentication service, inventory authority, award engine,
//! purchase-order executor, or payment processor.

use std::collections::{BTreeMap, BTreeSet};

pub use super::{EvidenceOrigin, EvidenceStatus};

const BPS_SCALE: u64 = 10_000;
const MAX_EXACT_OFFERS: usize = 16;
const MAX_SEARCH_NODES: u64 = 500_000;
const MAX_FEASIBLE_PORTFOLIOS: usize = 2_000;

/// Versioned request and policy context. Units and currency must already be canonical.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PortfolioRequest {
    pub request_id: String,
    pub capability_id: String,
    pub capability_profile_digest: String,
    pub unit: String,
    pub currency: String,
    pub required_evidence_origin: EvidenceOrigin,
    pub evaluated_at: u64,
    pub required_quantity: u32,
    pub max_total_cost_minor: u64,
    pub deliver_by: u64,
    pub allow_split: bool,
    pub max_suppliers: usize,
    /// Minimum number of suppliers with pairwise-disjoint declared failure-domain sets.
    pub minimum_independent_sources: usize,
    /// Maximum share of the requested quantity assigned to any one supplier, in basis points.
    pub max_supplier_share_bps: u16,
    pub max_quote_age_secs: u64,
    pub max_capacity_age_secs: u64,
    pub max_qualification_age_secs: u64,
    pub max_dependency_age_secs: u64,
    pub policy_id: String,
    pub policy_digest: String,
    pub scenario_set_id: String,
    pub scenario_set_digest: String,
    /// Digest of the exact candidate/scenario input manifest.
    pub input_manifest_digest: String,
    /// Exact bound on recursive search nodes. Exceeding it returns no partial result.
    pub max_search_nodes: u64,
    /// Bound on feasible allocations retained for frontier analysis.
    pub max_feasible_portfolios: usize,
}

impl PortfolioRequest {
    fn validate(&self, scenario_count: usize) -> Result<(), PortfolioError> {
        if [
            self.request_id.as_str(),
            self.capability_id.as_str(),
            self.capability_profile_digest.as_str(),
            self.unit.as_str(),
            self.currency.as_str(),
            self.policy_id.as_str(),
            self.policy_digest.as_str(),
            self.scenario_set_id.as_str(),
            self.scenario_set_digest.as_str(),
            self.input_manifest_digest.as_str(),
        ]
        .iter()
        .any(|value| value.trim().is_empty())
            || self.required_evidence_origin == EvidenceOrigin::Unknown
            || self.required_quantity == 0
            || self.deliver_by < self.evaluated_at
            || self.max_suppliers == 0
            || self.minimum_independent_sources == 0
            || self.minimum_independent_sources > if self.allow_split { self.max_suppliers } else { 1 }
            || self.minimum_independent_sources > MAX_EXACT_OFFERS
            || self.max_supplier_share_bps == 0
            || u64::from(self.max_supplier_share_bps) > BPS_SCALE
            || self.max_search_nodes == 0
            || self.max_search_nodes > MAX_SEARCH_NODES
            || self.max_feasible_portfolios == 0
            || self.max_feasible_portfolios > MAX_FEASIBLE_PORTFOLIOS
            || (scenario_count > 0 && self.scenario_set_digest.trim().is_empty())
        {
            return Err(PortfolioError::InvalidRequest);
        }
        Ok(())
    }
}

/// Candidate projection from an upstream eligibility/evidence process.
///
/// Status, origin, timestamps and references are input assertions: this module
/// checks their consistency but does not authenticate them or recompute digests.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PortfolioOffer {
    pub offer_id: String,
    pub supplier_id: String,
    pub source_uri: String,
    pub source_payload_digest: String,
    pub extraction_profile_id: String,
    pub qualification_receipt_id: String,
    pub capability_id: String,
    pub capability_profile_digest: String,
    pub unit: String,
    pub currency: String,

    pub qualification_status: EvidenceStatus,
    pub qualification_origin: EvidenceOrigin,
    pub qualification_observed_at: u64,
    pub qualification_valid_until: u64,

    pub quote_status: EvidenceStatus,
    pub quote_origin: EvidenceOrigin,
    pub quote_observed_at: u64,
    pub quote_valid_until: u64,

    pub capacity_status: EvidenceStatus,
    pub capacity_origin: EvidenceOrigin,
    pub capacity_pool_id: String,
    pub capacity_snapshot_digest: String,
    /// Shared ceiling for all offers referencing this capacity pool.
    pub capacity_pool_quantity: u32,
    /// A source may expose multiple offers against the same pool, but pool capacity is counted once.
    pub max_allocatable_quantity: u32,
    pub capacity_observed_at: u64,
    pub capacity_valid_until: u64,

    pub unit_cost_minor: u64,
    /// Charged once if this offer receives a non-zero allocation.
    pub fixed_order_cost_minor: u64,
    pub minimum_order_quantity: u32,
    /// Valid non-zero quantities are min, min + increment, min + 2*increment, ...
    pub order_increment: u32,
    pub arrival_at: u64,

    /// Failure-domain tags may describe supplier parent, plant, critical sub-tier,
    /// route, port, utility dependency or region. They are not inferred here.
    pub failure_domains: Vec<String>,
    pub failure_domain_status: EvidenceStatus,
    pub failure_domain_origin: EvidenceOrigin,
    pub failure_domain_observed_at: u64,
    pub failure_domain_valid_until: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DisruptionScenario {
    pub scenario_id: String,
    /// Any matching tag means the allocation from that offer is unavailable in this scenario.
    pub failed_domains: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortfolioError {
    InvalidRequest,
    DuplicateOfferId,
    DuplicateScenarioId,
    InconsistentCapacityPool,
    InvalidScenario,
    ExactSearchBoundExceeded,
    SearchSpaceExceeded,
    ArithmeticOverflow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RejectionReason {
    InvalidIdentityOrProvenance,
    CapabilityMismatch,
    CapabilityProfileMismatch,
    UnitMismatch,
    CurrencyMismatch,
    QualificationNotVerified,
    QualificationOriginMismatch,
    QualificationFromFuture,
    QualificationStale,
    QualificationExpired,
    QuoteNotVerified,
    QuoteOriginMismatch,
    QuoteFromFuture,
    QuoteStale,
    QuoteExpired,
    CapacityNotVerified,
    CapacityOriginMismatch,
    CapacityFromFuture,
    CapacityStale,
    CapacityExpired,
    NoCapacity,
    InvalidLotPolicy,
    LotExceedsOfferCapacity,
    CannotMeetDeadline,
    DependencyEvidenceNotVerified,
    DependencyOriginMismatch,
    DependencyFromFuture,
    DependencyStale,
    DependencyExpired,
    MissingFailureDomain,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RejectedOffer {
    pub offer_id: String,
    pub supplier_id: String,
    pub reason: RejectionReason,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct PortfolioAllocation {
    pub offer_id: String,
    pub supplier_id: String,
    pub qualification_receipt_id: String,
    pub source_uri: String,
    pub source_payload_digest: String,
    pub capacity_pool_id: String,
    pub capacity_snapshot_digest: String,
    pub quantity: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ScenarioShortage {
    pub scenario_id: String,
    pub unavailable_quantity: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Portfolio {
    pub allocations: Vec<PortfolioAllocation>,
    pub total_cost_minor: u64,
    pub latest_arrival_at: u64,
    /// Maximum allocated quantity made unavailable by any supplied scenario.
    /// This is a scenario-set result, not an expected loss or probability estimate.
    pub worst_case_unavailable_quantity: u32,
    pub scenario_shortages: Vec<ScenarioShortage>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PortfolioReport {
    pub evaluator_profile: &'static str,
    pub request_id: String,
    pub evaluated_at: u64,
    pub capability_id: String,
    pub capability_profile_digest: String,
    pub unit: String,
    pub currency: String,
    pub required_quantity: u32,
    pub required_evidence_origin: EvidenceOrigin,
    pub minimum_independent_sources: usize,
    pub policy_id: String,
    pub policy_digest: String,
    pub scenario_set_id: String,
    pub scenario_set_digest: String,
    pub input_manifest_digest: String,
    pub input_offer_count: usize,
    pub eligible_offer_count: usize,
    pub rejected_offers: Vec<RejectedOffer>,
    pub feasible_portfolio_count: usize,
    pub search_nodes: u64,
    pub resilience_assessed: bool,
    /// Full set of non-dominated feasible allocations; no arbitrary winner is selected.
    pub pareto_frontier: Vec<Portfolio>,
}

const PORTFOLIO_EVALUATOR_PROFILE: &str = "mycelix:portfolio-exact-frontier:v1";

#[derive(Clone, Debug, PartialEq, Eq)]
struct CapacityPoolSnapshot {
    quantity: u32,
    observed_at: u64,
    valid_until: u64,
    snapshot_digest: String,
    status: EvidenceStatus,
    origin: EvidenceOrigin,
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn offer_rejection(
    request: &PortfolioRequest,
    offer: &PortfolioOffer,
    dependency_evidence_required: bool,
) -> Option<RejectionReason> {
    if !non_empty(&offer.offer_id)
        || !non_empty(&offer.supplier_id)
        || !non_empty(&offer.source_uri)
        || !non_empty(&offer.source_payload_digest)
        || !non_empty(&offer.extraction_profile_id)
        || !non_empty(&offer.qualification_receipt_id)
        || !non_empty(&offer.capacity_pool_id)
        || !non_empty(&offer.capacity_snapshot_digest)
    {
        return Some(RejectionReason::InvalidIdentityOrProvenance);
    }
    if offer.capability_id != request.capability_id {
        return Some(RejectionReason::CapabilityMismatch);
    }
    if offer.capability_profile_digest != request.capability_profile_digest {
        return Some(RejectionReason::CapabilityProfileMismatch);
    }
    if offer.unit != request.unit {
        return Some(RejectionReason::UnitMismatch);
    }
    if offer.currency != request.currency {
        return Some(RejectionReason::CurrencyMismatch);
    }

    if offer.qualification_status != EvidenceStatus::Verified {
        return Some(RejectionReason::QualificationNotVerified);
    }
    if offer.qualification_origin != request.required_evidence_origin {
        return Some(RejectionReason::QualificationOriginMismatch);
    }
    if offer.qualification_observed_at > request.evaluated_at {
        return Some(RejectionReason::QualificationFromFuture);
    }
    if request.evaluated_at - offer.qualification_observed_at
        > request.max_qualification_age_secs
    {
        return Some(RejectionReason::QualificationStale);
    }
    if offer.qualification_valid_until < request.evaluated_at {
        return Some(RejectionReason::QualificationExpired);
    }

    if offer.quote_status != EvidenceStatus::Verified {
        return Some(RejectionReason::QuoteNotVerified);
    }
    if offer.quote_origin != request.required_evidence_origin {
        return Some(RejectionReason::QuoteOriginMismatch);
    }
    if offer.quote_observed_at > request.evaluated_at {
        return Some(RejectionReason::QuoteFromFuture);
    }
    if request.evaluated_at - offer.quote_observed_at > request.max_quote_age_secs {
        return Some(RejectionReason::QuoteStale);
    }
    if offer.quote_valid_until < request.evaluated_at {
        return Some(RejectionReason::QuoteExpired);
    }

    if offer.capacity_status != EvidenceStatus::Verified {
        return Some(RejectionReason::CapacityNotVerified);
    }
    if offer.capacity_origin != request.required_evidence_origin {
        return Some(RejectionReason::CapacityOriginMismatch);
    }
    if offer.capacity_observed_at > request.evaluated_at {
        return Some(RejectionReason::CapacityFromFuture);
    }
    if request.evaluated_at - offer.capacity_observed_at > request.max_capacity_age_secs {
        return Some(RejectionReason::CapacityStale);
    }
    if offer.capacity_valid_until < request.evaluated_at {
        return Some(RejectionReason::CapacityExpired);
    }
    if offer.capacity_pool_quantity == 0 || offer.max_allocatable_quantity == 0 {
        return Some(RejectionReason::NoCapacity);
    }
    if offer.minimum_order_quantity == 0 || offer.order_increment == 0 {
        return Some(RejectionReason::InvalidLotPolicy);
    }
    if offer.minimum_order_quantity > offer.max_allocatable_quantity.min(offer.capacity_pool_quantity) {
        return Some(RejectionReason::LotExceedsOfferCapacity);
    }
    if offer.arrival_at > request.deliver_by {
        return Some(RejectionReason::CannotMeetDeadline);
    }

    if dependency_evidence_required {
        if offer.failure_domain_status != EvidenceStatus::Verified {
            return Some(RejectionReason::DependencyEvidenceNotVerified);
        }
        if offer.failure_domain_origin != request.required_evidence_origin {
            return Some(RejectionReason::DependencyOriginMismatch);
        }
        if offer.failure_domain_observed_at > request.evaluated_at {
            return Some(RejectionReason::DependencyFromFuture);
        }
        if request.evaluated_at - offer.failure_domain_observed_at
            > request.max_dependency_age_secs
        {
            return Some(RejectionReason::DependencyStale);
        }
        if offer.failure_domain_valid_until < request.evaluated_at {
            return Some(RejectionReason::DependencyExpired);
        }
        if offer.failure_domains.is_empty() || offer.failure_domains.iter().any(|domain| !non_empty(domain)) {
            return Some(RejectionReason::MissingFailureDomain);
        }
    }
    None
}

fn allocation_choices(offer: &PortfolioOffer, request_quantity: u32) -> Vec<u32> {
    let max_quantity = offer
        .max_allocatable_quantity
        .min(offer.capacity_pool_quantity)
        .min(request_quantity);
    let mut choices = vec![0];
    let mut quantity = offer.minimum_order_quantity;
    while quantity <= max_quantity {
        choices.push(quantity);
        let Some(next) = quantity.checked_add(offer.order_increment) else {
            break;
        };
        quantity = next;
    }
    choices
}

struct SearchState<'a> {
    request: &'a PortfolioRequest,
    offers: &'a [PortfolioOffer],
    scenarios: &'a [DisruptionScenario],
    choices: Vec<Vec<u32>>,
    nodes: u64,
    feasible: Vec<Portfolio>,
}

impl SearchState<'_> {
    fn recurse(&mut self, index: usize, allocations: &mut [u32]) -> Result<(), PortfolioError> {
        self.nodes = self.nodes.checked_add(1).ok_or(PortfolioError::ArithmeticOverflow)?;
        if self.nodes > self.request.max_search_nodes {
            return Err(PortfolioError::SearchSpaceExceeded);
        }

        if index == self.offers.len() {
            let total_quantity = allocations.iter().try_fold(0u64, |sum, quantity| {
                sum.checked_add(u64::from(*quantity))
                    .ok_or(PortfolioError::ArithmeticOverflow)
            })?;
            if total_quantity != u64::from(self.request.required_quantity) {
                return Ok(());
            }
            if !self.portfolio_constraints_hold(allocations)? {
                return Ok(());
            }
            let Some(portfolio) = self.build_portfolio(allocations)? else {
                return Ok(());
            };
            if self.feasible.len() >= self.request.max_feasible_portfolios {
                return Err(PortfolioError::SearchSpaceExceeded);
            }
            self.feasible.push(portfolio);
            return Ok(());
        }

        let choices = self.choices[index].clone();
        for quantity in choices {
            allocations[index] = quantity;
            if self.partial_constraints_hold(index, allocations)? {
                self.recurse(index + 1, allocations)?;
            }
        }
        allocations[index] = 0;
        Ok(())
    }

    fn partial_constraints_hold(
        &self,
        index: usize,
        allocations: &[u32],
    ) -> Result<bool, PortfolioError> {
        let mut total_quantity = 0u64;
        let mut supplier_quantities: BTreeMap<String, u64> = BTreeMap::new();
        let mut pool_quantities: BTreeMap<String, u64> = BTreeMap::new();
        let mut pool_caps: BTreeMap<String, u32> = BTreeMap::new();

        for (offer, quantity) in self.offers.iter().zip(allocations.iter()).take(index + 1) {
            total_quantity = total_quantity
                .checked_add(u64::from(*quantity))
                .ok_or(PortfolioError::ArithmeticOverflow)?;
            if *quantity == 0 {
                continue;
            }
            let supplier_total = supplier_quantities.entry(offer.supplier_id.clone()).or_default();
            *supplier_total = supplier_total
                .checked_add(u64::from(*quantity))
                .ok_or(PortfolioError::ArithmeticOverflow)?;
            let pool_total = pool_quantities.entry(offer.capacity_pool_id.clone()).or_default();
            *pool_total = pool_total
                .checked_add(u64::from(*quantity))
                .ok_or(PortfolioError::ArithmeticOverflow)?;
            pool_caps.insert(offer.capacity_pool_id.clone(), offer.capacity_pool_quantity);
        }

        if total_quantity > u64::from(self.request.required_quantity) {
            return Ok(false);
        }
        if supplier_quantities.len() > self.effective_max_suppliers() {
            return Ok(false);
        }
        for (_supplier, quantity) in &supplier_quantities {
            if quantity.checked_mul(BPS_SCALE).ok_or(PortfolioError::ArithmeticOverflow)?
                > u64::from(self.request.required_quantity)
                    .checked_mul(u64::from(self.request.max_supplier_share_bps))
                    .ok_or(PortfolioError::ArithmeticOverflow)?
            {
                return Ok(false);
            }
        }
        for (pool, quantity) in pool_quantities {
            if quantity > u64::from(*pool_caps.get(&pool).expect("pool cap inserted with usage")) {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn effective_max_suppliers(&self) -> usize {
        if self.request.allow_split {
            self.request.max_suppliers
        } else {
            1
        }
    }

    fn minimum_independent_sources_hold(&self, allocations: &[u32]) -> bool {
        if self.request.minimum_independent_sources <= 1 {
            return true;
        }
        let mut groups: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
        for (offer, quantity) in self.offers.iter().zip(allocations.iter()) {
            if *quantity == 0 {
                continue;
            }
            let domains = groups.entry(offer.supplier_id.clone()).or_default();
            // Supplier and capacity-pool identities are always common-cause domains,
            // even if upstream mapping omits these obvious links.
            domains.insert(format!("supplier:{}", offer.supplier_id));
            domains.insert(format!("capacity-pool:{}", offer.capacity_pool_id));
            domains.extend(offer.failure_domains.iter().cloned());
        }
        let group_sets: Vec<BTreeSet<String>> = groups.into_values().collect();
        can_choose_independent_groups(
            &group_sets,
            0,
            self.request.minimum_independent_sources,
            &BTreeSet::new(),
        )
    }

    fn portfolio_constraints_hold(&self, allocations: &[u32]) -> Result<bool, PortfolioError> {
        if !self.minimum_independent_sources_hold(allocations) {
            return Ok(false);
        }
        let mut total_cost = 0u64;
        for (offer, quantity) in self.offers.iter().zip(allocations.iter()) {
            if *quantity == 0 {
                continue;
            }
            let variable_cost = offer
                .unit_cost_minor
                .checked_mul(u64::from(*quantity))
                .ok_or(PortfolioError::ArithmeticOverflow)?;
            let order_cost = variable_cost
                .checked_add(offer.fixed_order_cost_minor)
                .ok_or(PortfolioError::ArithmeticOverflow)?;
            total_cost = total_cost
                .checked_add(order_cost)
                .ok_or(PortfolioError::ArithmeticOverflow)?;
        }
        Ok(total_cost <= self.request.max_total_cost_minor)
    }

    fn build_portfolio(&self, allocations: &[u32]) -> Result<Option<Portfolio>, PortfolioError> {
        let mut total_cost = 0u64;
        let mut latest_arrival_at = 0u64;
        let mut built_allocations = Vec::new();

        for (offer, quantity) in self.offers.iter().zip(allocations.iter()) {
            if *quantity == 0 {
                continue;
            }
            let variable_cost = offer
                .unit_cost_minor
                .checked_mul(u64::from(*quantity))
                .ok_or(PortfolioError::ArithmeticOverflow)?;
            let order_cost = variable_cost
                .checked_add(offer.fixed_order_cost_minor)
                .ok_or(PortfolioError::ArithmeticOverflow)?;
            total_cost = total_cost
                .checked_add(order_cost)
                .ok_or(PortfolioError::ArithmeticOverflow)?;
            latest_arrival_at = latest_arrival_at.max(offer.arrival_at);
            built_allocations.push(PortfolioAllocation {
                offer_id: offer.offer_id.clone(),
                supplier_id: offer.supplier_id.clone(),
                qualification_receipt_id: offer.qualification_receipt_id.clone(),
                source_uri: offer.source_uri.clone(),
                source_payload_digest: offer.source_payload_digest.clone(),
                capacity_pool_id: offer.capacity_pool_id.clone(),
                capacity_snapshot_digest: offer.capacity_snapshot_digest.clone(),
                quantity: *quantity,
            });
        }
        if total_cost > self.request.max_total_cost_minor {
            return Ok(None);
        }

        let mut scenario_shortages = Vec::with_capacity(self.scenarios.len());
        let mut worst_case_unavailable_quantity = 0u32;
        for scenario in self.scenarios {
            let failed: BTreeSet<&str> =
                scenario.failed_domains.iter().map(String::as_str).collect();
            let mut unavailable = 0u32;
            for (offer, quantity) in self.offers.iter().zip(allocations.iter()) {
                if *quantity == 0 {
                    continue;
                }
                if failed.contains(format!("supplier:{}", offer.supplier_id).as_str())
                    || failed.contains(format!("capacity-pool:{}", offer.capacity_pool_id).as_str())
                    || offer.failure_domains.iter().any(|domain| failed.contains(domain.as_str())) {
                    unavailable = unavailable
                        .checked_add(*quantity)
                        .ok_or(PortfolioError::ArithmeticOverflow)?;
                }
            }
            worst_case_unavailable_quantity =
                worst_case_unavailable_quantity.max(unavailable);
            scenario_shortages.push(ScenarioShortage {
                scenario_id: scenario.scenario_id.clone(),
                unavailable_quantity: unavailable,
            });
        }
        scenario_shortages.sort_by(|a, b| a.scenario_id.cmp(&b.scenario_id));
        built_allocations.sort();

        Ok(Some(Portfolio {
            allocations: built_allocations,
            total_cost_minor: total_cost,
            latest_arrival_at,
            worst_case_unavailable_quantity,
            scenario_shortages,
        }))
    }
}

fn can_choose_independent_groups(
    groups: &[BTreeSet<String>],
    start: usize,
    remaining: usize,
    occupied: &BTreeSet<String>,
) -> bool {
    if remaining == 0 {
        return true;
    }
    if groups.len().saturating_sub(start) < remaining {
        return false;
    }
    for index in start..groups.len() {
        if groups[index].is_disjoint(occupied) {
            let mut next_occupied = occupied.clone();
            next_occupied.extend(groups[index].iter().cloned());
            if can_choose_independent_groups(groups, index + 1, remaining - 1, &next_occupied) {
                return true;
            }
        }
    }
    false
}

fn dominates(left: &Portfolio, right: &Portfolio) -> bool {
    let no_worse = left.total_cost_minor <= right.total_cost_minor
        && left.latest_arrival_at <= right.latest_arrival_at
        && left.worst_case_unavailable_quantity <= right.worst_case_unavailable_quantity;
    let strictly_better = left.total_cost_minor < right.total_cost_minor
        || left.latest_arrival_at < right.latest_arrival_at
        || left.worst_case_unavailable_quantity < right.worst_case_unavailable_quantity;
    no_worse && strictly_better
}

/// Enumerates small allocation spaces exactly. On a search-bound hit, returns an
/// error and no partial report; callers must not present a partial frontier as complete.
pub fn evaluate_portfolios(
    request: &PortfolioRequest,
    offers: &[PortfolioOffer],
    scenarios: &[DisruptionScenario],
) -> Result<PortfolioReport, PortfolioError> {
    request.validate(scenarios.len())?;
    if offers.len() > MAX_EXACT_OFFERS {
        return Err(PortfolioError::ExactSearchBoundExceeded);
    }

    let mut scenario_ids = BTreeSet::new();
    for scenario in scenarios {
        if !non_empty(&scenario.scenario_id)
            || scenario.failed_domains.iter().all(|domain| !non_empty(domain))
        {
            return Err(PortfolioError::InvalidScenario);
        }
        if !scenario_ids.insert(scenario.scenario_id.as_str()) {
            return Err(PortfolioError::DuplicateScenarioId);
        }
    }

    let mut offer_ids = BTreeSet::new();
    let mut pool_snapshots: BTreeMap<String, CapacityPoolSnapshot> = BTreeMap::new();
    for offer in offers {
        if !offer_ids.insert(offer.offer_id.as_str()) {
            return Err(PortfolioError::DuplicateOfferId);
        }
        if non_empty(&offer.capacity_pool_id) {
            let snapshot = CapacityPoolSnapshot {
                quantity: offer.capacity_pool_quantity,
                observed_at: offer.capacity_observed_at,
                valid_until: offer.capacity_valid_until,
                snapshot_digest: offer.capacity_snapshot_digest.clone(),
                status: offer.capacity_status,
                origin: offer.capacity_origin,
            };
            if let Some(existing) = pool_snapshots.get(&offer.capacity_pool_id) {
                if existing != &snapshot {
                    return Err(PortfolioError::InconsistentCapacityPool);
                }
            } else {
                pool_snapshots.insert(offer.capacity_pool_id.clone(), snapshot);
            }
        }
    }

    let resilience_assessed = !scenarios.is_empty();
    let dependency_evidence_required = resilience_assessed || request.minimum_independent_sources > 1;
    let mut eligible = Vec::new();
    let mut rejected_offers = Vec::new();
    for offer in offers {
        if let Some(reason) = offer_rejection(request, offer, dependency_evidence_required) {
            rejected_offers.push(RejectedOffer {
                offer_id: offer.offer_id.clone(),
                supplier_id: offer.supplier_id.clone(),
                reason,
            });
        } else {
            eligible.push(offer.clone());
        }
    }
    eligible.sort_by(|a, b| a.offer_id.cmp(&b.offer_id));
    rejected_offers.sort_by(|a, b| a.offer_id.cmp(&b.offer_id));

    let choices = eligible
        .iter()
        .map(|offer| allocation_choices(offer, request.required_quantity))
        .collect();
    let mut search = SearchState {
        request,
        offers: &eligible,
        scenarios,
        choices,
        nodes: 0,
        feasible: Vec::new(),
    };
    let mut allocations = vec![0; eligible.len()];
    search.recurse(0, &mut allocations)?;

    let feasible_portfolio_count = search.feasible.len();
    let mut frontier = Vec::new();
    for (candidate_index, candidate) in search.feasible.iter().enumerate() {
        let is_dominated = search
            .feasible
            .iter()
            .enumerate()
            .any(|(other_index, other)| {
                candidate_index != other_index && dominates(other, candidate)
            });
        if !is_dominated {
            frontier.push(candidate.clone());
        }
    }
    frontier.sort_by(|left, right| {
        left.total_cost_minor
            .cmp(&right.total_cost_minor)
            .then_with(|| left.latest_arrival_at.cmp(&right.latest_arrival_at))
            .then_with(|| {
                left.worst_case_unavailable_quantity
                    .cmp(&right.worst_case_unavailable_quantity)
            })
            .then_with(|| left.allocations.cmp(&right.allocations))
    });

    Ok(PortfolioReport {
        evaluator_profile: PORTFOLIO_EVALUATOR_PROFILE,
        request_id: request.request_id.clone(),
        evaluated_at: request.evaluated_at,
        capability_id: request.capability_id.clone(),
        capability_profile_digest: request.capability_profile_digest.clone(),
        unit: request.unit.clone(),
        currency: request.currency.clone(),
        required_quantity: request.required_quantity,
        required_evidence_origin: request.required_evidence_origin,
        policy_id: request.policy_id.clone(),
        policy_digest: request.policy_digest.clone(),
        scenario_set_id: request.scenario_set_id.clone(),
        scenario_set_digest: request.scenario_set_digest.clone(),
        input_manifest_digest: request.input_manifest_digest.clone(),
        input_offer_count: offers.len(),
        eligible_offer_count: eligible.len(),
        rejected_offers,
        feasible_portfolio_count,
        search_nodes: search.nodes,
        resilience_assessed,
        pareto_frontier: frontier,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request() -> PortfolioRequest {
        PortfolioRequest {
            request_id: "req-1".into(),
            capability_id: "capability:fastener".into(),
            capability_profile_digest: "sha256:profile-v1".into(),
            unit: "each".into(),
            currency: "USD".into(),
            required_evidence_origin: EvidenceOrigin::SyntheticSimulation,
            evaluated_at: 100,
            required_quantity: 10,
            max_total_cost_minor: 200,
            deliver_by: 130,
            allow_split: true,
            max_suppliers: 2,
            max_supplier_share_bps: 6_000,
            minimum_independent_sources: 2,
            max_quote_age_secs: 20,
            max_capacity_age_secs: 20,
            max_qualification_age_secs: 50,
            max_dependency_age_secs: 50,
            policy_id: "policy:test-v1".into(),
            policy_digest: "sha256:policy-v1".into(),
            scenario_set_id: "scenarios:test-v1".into(),
            scenario_set_digest: "sha256:scenarios-v1".into(),
            input_manifest_digest: "sha256:manifest-v1".into(),
            max_search_nodes: 50_000,
            max_feasible_portfolios: 2_000,
        }
    }

    fn offer(id: &str, supplier: &str, pool: &str, capacity: u32, cost: u64, arrival: u64, domain: &str) -> PortfolioOffer {
        PortfolioOffer {
            offer_id: id.into(),
            supplier_id: supplier.into(),
            source_uri: format!("https://fixture.invalid/{id}"),
            source_payload_digest: format!("sha256:source-{id}"),
            extraction_profile_id: "extractor:fixture-v1".into(),
            qualification_receipt_id: format!("qualification:{id}"),
            capability_id: "capability:fastener".into(),
            capability_profile_digest: "sha256:profile-v1".into(),
            unit: "each".into(),
            currency: "USD".into(),

            qualification_status: EvidenceStatus::Verified,
            qualification_origin: EvidenceOrigin::SyntheticSimulation,
            qualification_observed_at: 90,
            qualification_valid_until: 150,

            quote_status: EvidenceStatus::Verified,
            quote_origin: EvidenceOrigin::SyntheticSimulation,
            quote_observed_at: 95,
            quote_valid_until: 140,

            capacity_status: EvidenceStatus::Verified,
            capacity_origin: EvidenceOrigin::SyntheticSimulation,
            capacity_pool_id: pool.into(),
            capacity_snapshot_digest: format!("sha256:capacity-{pool}"),
            capacity_pool_quantity: capacity,
            max_allocatable_quantity: capacity,
            capacity_observed_at: 98,
            capacity_valid_until: 130,

            unit_cost_minor: cost,
            fixed_order_cost_minor: 0,
            minimum_order_quantity: 1,
            order_increment: 1,
            arrival_at: arrival,

            failure_domains: vec![domain.into()],
            failure_domain_status: EvidenceStatus::Verified,
            failure_domain_origin: EvidenceOrigin::SyntheticSimulation,
            failure_domain_observed_at: 90,
            failure_domain_valid_until: 150,
        }
    }

    #[test]
    fn split_award_uses_two_partial_capacity_suppliers() {
        let request = request();
        let offers = vec![
            offer("offer-a", "supplier-a", "pool-a", 6, 1, 110, "region-a"),
            offer("offer-b", "supplier-b", "pool-b", 6, 2, 120, "region-b"),
        ];
        let scenarios = vec![
            DisruptionScenario { scenario_id: "fail-a".into(), failed_domains: vec!["region-a".into()] },
            DisruptionScenario { scenario_id: "fail-b".into(), failed_domains: vec!["region-b".into()] },
        ];
        let report = evaluate_portfolios(&request, &offers, &scenarios).unwrap();
        assert_eq!(report.eligible_offer_count, 2);
        assert!(report.feasible_portfolio_count > 0);
        assert!(!report.pareto_frontier.is_empty());
        assert!(report.pareto_frontier.iter().all(|p| {
            p.allocations.iter().map(|a| u64::from(a.quantity)).sum::<u64>() == 10
        }));
        assert!(report.pareto_frontier.iter().any(|p| {
            p.total_cost_minor == 14
                && p.allocations.iter().any(|a| a.supplier_id == "supplier-a" && a.quantity == 6)
                && p.allocations.iter().any(|a| a.supplier_id == "supplier-b" && a.quantity == 4)
        }));
        assert!(report.pareto_frontier.iter().all(|p| p.worst_case_unavailable_quantity >= 4));
    }

    #[test]
    fn single_supplier_policy_rejects_a_required_split() {
        let mut request = request();
        request.allow_split = false;
        request.max_suppliers = 1;
        request.minimum_independent_sources = 1;
        request.max_supplier_share_bps = 10_000;
        let offers = vec![
            offer("offer-a", "supplier-a", "pool-a", 6, 1, 110, "region-a"),
            offer("offer-b", "supplier-b", "pool-b", 6, 2, 120, "region-b"),
        ];
        let report = evaluate_portfolios(&request, &offers, &[]).unwrap();
        assert_eq!(report.feasible_portfolio_count, 0);
        assert!(report.pareto_frontier.is_empty());
    }

    #[test]
    fn shared_capacity_pool_cannot_be_counted_twice() {
        let request = request();
        let offers = vec![
            offer("offer-a", "supplier-a", "shared-pool", 6, 1, 110, "region-a"),
            offer("offer-b", "supplier-a", "shared-pool", 6, 2, 110, "region-a"),
        ];
        let report = evaluate_portfolios(&request, &offers, &[]).unwrap();
        assert_eq!(report.feasible_portfolio_count, 0);
    }

    #[test]
    fn unverified_dependency_mapping_is_not_resilience_evidence() {
        let request = request();
        let mut candidate = offer("offer-a", "supplier-a", "pool-a", 10, 1, 110, "region-a");
        candidate.failure_domain_status = EvidenceStatus::Unknown;
        let report = evaluate_portfolios(&request, &[candidate], &[DisruptionScenario {
            scenario_id: "site-outage".into(),
            failed_domains: vec!["region-a".into()],
        }]).unwrap();
        assert_eq!(report.eligible_offer_count, 0);
        assert_eq!(report.rejected_offers[0].reason, RejectionReason::DependencyEvidenceNotVerified);
    }

    #[test]
    fn stale_capacity_is_rejected_not_promoted() {
        let mut request = request();
        request.max_capacity_age_secs = 1;
        let offer = offer("offer-a", "supplier-a", "pool-a", 10, 1, 110, "region-a");
        let report = evaluate_portfolios(&request, &[offer], &[]).unwrap();
        assert_eq!(report.eligible_offer_count, 0);
        assert_eq!(report.rejected_offers[0].reason, RejectionReason::CapacityStale);
    }

    #[test]
    fn two_supplier_frontier_matches_independent_exhaustive_oracle() {
        let req = request();
        let offers = vec![
            offer("offer-a", "supplier-a", "pool-a", 6, 1, 110, "region-a"),
            offer("offer-b", "supplier-b", "pool-b", 6, 2, 120, "region-b"),
        ];
        let scenarios = vec![
            DisruptionScenario { scenario_id: "fail-a".into(), failed_domains: vec!["region-a".into()] },
            DisruptionScenario { scenario_id: "fail-b".into(), failed_domains: vec!["region-b".into()] },
        ];

        // Independent brute-force oracle for this two-source fixture. It directly
        // enumerates q_a + q_b = demand and computes the three objective values;
        // it does not call the production enumerator or dominance helper.
        let mut all_objectives: Vec<(u64, u64, u32)> = Vec::new();
        for q_a in 0..=6u32 {
            for q_b in 0..=6u32 {
                if q_a + q_b != req.required_quantity {
                    continue;
                }
                if q_a > 0 && q_a * 10_000 > req.required_quantity * u32::from(req.max_supplier_share_bps) {
                    continue;
                }
                if q_b > 0 && q_b * 10_000 > req.required_quantity * u32::from(req.max_supplier_share_bps) {
                    continue;
                }
                let cost = u64::from(q_a) + 2 * u64::from(q_b);
                if cost > req.max_total_cost_minor {
                    continue;
                }
                let latest = if q_a > 0 && q_b > 0 { 120 } else if q_a > 0 { 110 } else { 120 };
                let worst_unavailable = q_a.max(q_b);
                all_objectives.push((cost, latest, worst_unavailable));
            }
        }
        let mut expected: Vec<(u64, u64, u32)> = all_objectives
            .iter()
            .copied()
            .filter(|candidate| {
                !all_objectives.iter().any(|other| {
                    let no_worse = other.0 <= candidate.0
                        && other.1 <= candidate.1
                        && other.2 <= candidate.2;
                    let strictly_better = other.0 < candidate.0
                        || other.1 < candidate.1
                        || other.2 < candidate.2;
                    no_worse && strictly_better
                })
            })
            .collect();
        expected.sort();

        let report = evaluate_portfolios(&req, &offers, &scenarios).unwrap();
        let mut actual: Vec<(u64, u64, u32)> = report
            .pareto_frontier
            .iter()
            .map(|portfolio| (
                portfolio.total_cost_minor,
                portfolio.latest_arrival_at,
                portfolio.worst_case_unavailable_quantity,
            ))
            .collect();
        actual.sort();
        assert_eq!(actual, expected);
        assert_eq!(expected, vec![(14, 120, 6), (15, 120, 5)]);
    }

    #[test]
    fn duplicate_offer_ids_fail_closed() {
        let request = request();
        let candidate = offer("duplicate", "supplier-a", "pool-a", 10, 1, 110, "region-a");
        let result = evaluate_portfolios(&request, &[candidate.clone(), candidate], &[]);
        assert_eq!(result, Err(PortfolioError::DuplicateOfferId));
    }
}
