//! Two-stage adaptive procurement recourse reference.
//!
//! Evaluates a pre-existing baseline allocation against declared disruption
//! scenarios and computes bounded, scenario-specific use of pre-qualified,
//! pre-committed flexible/backup capacity. This is a deterministic reference
//! model only: no supplier contact, contract change, order, stock mutation or
//! payment can be executed from this module.

use std::collections::{BTreeMap, BTreeSet};

use super::{EvidenceOrigin, EvidenceStatus};

const MAX_BASE_ALLOCATIONS: usize = 16;
const MAX_CONTINGENCY_OPTIONS: usize = 16;
const MAX_SCENARIOS: usize = 32;
const MAX_SEARCH_NODES: u64 = 500_000;
const MAX_RECOURSE_QUANTITY_CHOICES: u64 = 100_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SupplyMode {
    Flexible,
    Backup,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecourseRequest {
    pub request_id: String,
    pub capability_id: String,
    pub capability_profile_digest: String,
    pub unit: String,
    pub currency: String,
    pub required_evidence_origin: EvidenceOrigin,
    pub evaluated_at: u64,
    pub required_quantity: u32,
    pub deliver_by: u64,
    pub max_total_cost_minor: u64,
    pub max_qualification_age_secs: u64,
    pub max_quote_age_secs: u64,
    pub max_capacity_age_secs: u64,
    pub max_dependency_age_secs: u64,
    pub policy_id: String,
    pub policy_digest: String,
    pub scenario_set_id: String,
    pub scenario_set_digest: String,
    pub input_manifest_digest: String,
    /// Global recursive-node budget divided across all scenarios.
    pub max_search_nodes: u64,
}

impl RecourseRequest {
    fn validate(&self, scenario_count: usize) -> Result<(), RecourseError> {
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
            || scenario_count == 0
            || scenario_count > MAX_SCENARIOS
            || self.max_search_nodes < scenario_count as u64
            || self.max_search_nodes > MAX_SEARCH_NODES
        {
            return Err(RecourseError::InvalidRequest);
        }
        Ok(())
    }
}

/// Shared, versioned supply evidence. Fields are caller-supplied assertions:
/// this module checks consistency and age but does not authenticate the source,
/// signatures, credential issuer, or digest contents.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupplyEvidence {
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
    /// Shared physical/contractual capacity ceiling for this pool.
    pub capacity_pool_quantity: u32,
    pub capacity_observed_at: u64,
    pub capacity_valid_until: u64,

    pub failure_domains: Vec<String>,
    pub failure_domain_status: EvidenceStatus,
    pub failure_domain_origin: EvidenceOrigin,
    pub failure_domain_observed_at: u64,
    pub failure_domain_valid_until: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BaseAllocation {
    pub supply: SupplyEvidence,
    pub quantity: u32,
    pub max_allocatable_quantity: u32,
    pub minimum_order_quantity: u32,
    pub order_increment: u32,
    pub unit_cost_minor: u64,
    /// Paid once for a non-zero base allocation.
    pub fixed_order_cost_minor: u64,
    /// Absolute planned arrival time.
    pub arrival_at: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ContingencyOption {
    pub supply: SupplyEvidence,
    pub mode: SupplyMode,
    /// Maximum quantity that can be activated, before the shared pool constraint.
    pub max_activation_quantity: u32,
    pub minimum_order_quantity: u32,
    pub order_increment: u32,
    /// Time from the scenario observation to activation becoming effective.
    pub activation_lead_secs: u64,
    /// Transport/fulfilment time after activation.
    pub transit_secs: u64,
    pub unit_cost_minor: u64,
    /// Paid once if this option is actually activated in a scenario.
    pub activation_fee_minor: u64,
    /// Committed option/standby fee included in the baseline cost even if unused.
    pub reservation_fee_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecourseScenario {
    pub scenario_id: String,
    pub observed_at: u64,
    /// Empty means a no-disruption control. Reserved namespaces:
    /// supplier:<supplier_id>, capacity-pool:<capacity_pool_id>.
    pub failed_domains: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecourseError {
    InvalidRequest,
    InvalidScenario,
    DuplicateScenarioId,
    DuplicateOfferId,
    InvalidBaselinePlan,
    InvalidBaselineEvidence,
    InconsistentCapacityPool,
    CapacityPoolOvercommitted,
    BaseCommitmentOverBudget,
    ArithmeticOverflow,
    SearchSpaceExceeded,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SupplyRejection {
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
    DependencyEvidenceNotVerified,
    DependencyOriginMismatch,
    DependencyFromFuture,
    DependencyStale,
    DependencyExpired,
    MissingFailureDomain,
    InvalidLotPolicy,
    ActivationAfterDeadline,
    FailedInScenario,
    TimeOverflow,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct RecourseAllocation {
    pub offer_id: String,
    pub supplier_id: String,
    pub mode: SupplyMode,
    pub capacity_pool_id: String,
    pub capacity_snapshot_digest: String,
    pub qualification_receipt_id: String,
    pub source_uri: String,
    pub source_payload_digest: String,
    pub quantity: u32,
    pub activation_at: u64,
    pub arrival_at: u64,
    pub allocated_cost_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExcludedOption {
    pub offer_id: String,
    pub supplier_id: String,
    pub reason: SupplyRejection,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecourseScenarioResult {
    pub scenario_id: String,
    pub observed_at: u64,
    pub failed_domains: Vec<String>,
    pub base_lost_quantity: u32,
    pub surviving_base_quantity: u32,
    pub recovered_quantity: u32,
    pub unfilled_quantity: u32,
    pub recourse_cost_minor: u64,
    pub total_scenario_cost_minor: u64,
    pub latest_recovery_arrival_at: Option<u64>,
    pub flexible_quantity: u32,
    pub backup_quantity: u32,
    pub allocations: Vec<RecourseAllocation>,
    pub excluded_options: Vec<ExcludedOption>,
    pub search_nodes: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecourseReport {
    pub evaluator_profile: &'static str,
    pub decision_policy: &'static str,
    pub request_id: String,
    pub evaluated_at: u64,
    pub capability_id: String,
    pub capability_profile_digest: String,
    pub unit: String,
    pub currency: String,
    pub required_quantity: u32,
    pub policy_id: String,
    pub policy_digest: String,
    pub scenario_set_id: String,
    pub scenario_set_digest: String,
    pub input_manifest_digest: String,
    pub baseline: Vec<BaseAllocation>,
    pub contingency_option_ids: Vec<String>,
    pub baseline_and_reservation_cost_minor: u64,
    pub scenario_results: Vec<RecourseScenarioResult>,
    pub total_search_nodes: u64,
}

const EVALUATOR_PROFILE: &str = "mycelix:adaptive-recourse-v1";
const DECISION_POLICY: &str = "maximize-on-time-recovery-then-minimize-cost-then-arrival-v1";

#[derive(Clone, Debug, PartialEq, Eq)]
struct CapacityPoolSnapshot {
    quantity: u32,
    observed_at: u64,
    valid_until: u64,
    digest: String,
    status: EvidenceStatus,
    origin: EvidenceOrigin,
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn evidence_rejection(
    request: &RecourseRequest,
    supply: &SupplyEvidence,
    at: u64,
    dependency_required: bool,
) -> Option<SupplyRejection> {
    if !non_empty(&supply.offer_id)
        || !non_empty(&supply.supplier_id)
        || !non_empty(&supply.source_uri)
        || !non_empty(&supply.source_payload_digest)
        || !non_empty(&supply.extraction_profile_id)
        || !non_empty(&supply.qualification_receipt_id)
        || !non_empty(&supply.capacity_pool_id)
        || !non_empty(&supply.capacity_snapshot_digest)
    {
        return Some(SupplyRejection::InvalidIdentityOrProvenance);
    }
    if supply.capability_id != request.capability_id {
        return Some(SupplyRejection::CapabilityMismatch);
    }
    if supply.capability_profile_digest != request.capability_profile_digest {
        return Some(SupplyRejection::CapabilityProfileMismatch);
    }
    if supply.unit != request.unit {
        return Some(SupplyRejection::UnitMismatch);
    }
    if supply.currency != request.currency {
        return Some(SupplyRejection::CurrencyMismatch);
    }

    if supply.qualification_status != EvidenceStatus::Verified {
        return Some(SupplyRejection::QualificationNotVerified);
    }
    if supply.qualification_origin != request.required_evidence_origin {
        return Some(SupplyRejection::QualificationOriginMismatch);
    }
    if supply.qualification_observed_at > at {
        return Some(SupplyRejection::QualificationFromFuture);
    }
    if at - supply.qualification_observed_at > request.max_qualification_age_secs {
        return Some(SupplyRejection::QualificationStale);
    }
    if supply.qualification_valid_until < at {
        return Some(SupplyRejection::QualificationExpired);
    }

    if supply.quote_status != EvidenceStatus::Verified {
        return Some(SupplyRejection::QuoteNotVerified);
    }
    if supply.quote_origin != request.required_evidence_origin {
        return Some(SupplyRejection::QuoteOriginMismatch);
    }
    if supply.quote_observed_at > at {
        return Some(SupplyRejection::QuoteFromFuture);
    }
    if at - supply.quote_observed_at > request.max_quote_age_secs {
        return Some(SupplyRejection::QuoteStale);
    }
    if supply.quote_valid_until < at {
        return Some(SupplyRejection::QuoteExpired);
    }

    if supply.capacity_status != EvidenceStatus::Verified {
        return Some(SupplyRejection::CapacityNotVerified);
    }
    if supply.capacity_origin != request.required_evidence_origin {
        return Some(SupplyRejection::CapacityOriginMismatch);
    }
    if supply.capacity_observed_at > at {
        return Some(SupplyRejection::CapacityFromFuture);
    }
    if at - supply.capacity_observed_at > request.max_capacity_age_secs {
        return Some(SupplyRejection::CapacityStale);
    }
    if supply.capacity_valid_until < at {
        return Some(SupplyRejection::CapacityExpired);
    }
    if supply.capacity_pool_quantity == 0 {
        return Some(SupplyRejection::NoCapacity);
    }

    if dependency_required {
        if supply.failure_domain_status != EvidenceStatus::Verified {
            return Some(SupplyRejection::DependencyEvidenceNotVerified);
        }
        if supply.failure_domain_origin != request.required_evidence_origin {
            return Some(SupplyRejection::DependencyOriginMismatch);
        }
        if supply.failure_domain_observed_at > at {
            return Some(SupplyRejection::DependencyFromFuture);
        }
        if at - supply.failure_domain_observed_at > request.max_dependency_age_secs {
            return Some(SupplyRejection::DependencyStale);
        }
        if supply.failure_domain_valid_until < at {
            return Some(SupplyRejection::DependencyExpired);
        }
        if supply.failure_domains.iter().any(|domain| !non_empty(domain)) {
            return Some(SupplyRejection::MissingFailureDomain);
        }
        if supply.failure_domains.is_empty() {
            return Some(SupplyRejection::MissingFailureDomain);
        }
    }
    None
}

fn validate_base_lot(allocation: &BaseAllocation) -> bool {
    allocation.quantity > 0
        && allocation.quantity <= allocation.max_allocatable_quantity
        && allocation.minimum_order_quantity > 0
        && allocation.order_increment > 0
        && allocation.quantity >= allocation.minimum_order_quantity
        && (allocation.quantity - allocation.minimum_order_quantity)
            % allocation.order_increment == 0
}

fn capacity_pool_snapshot(supply: &SupplyEvidence) -> CapacityPoolSnapshot {
    CapacityPoolSnapshot {
        quantity: supply.capacity_pool_quantity,
        observed_at: supply.capacity_observed_at,
        valid_until: supply.capacity_valid_until,
        digest: supply.capacity_snapshot_digest.clone(),
        status: supply.capacity_status,
        origin: supply.capacity_origin,
    }
}

fn failed(
    supply: &SupplyEvidence,
    failed_domains: &BTreeSet<&str>,
) -> bool {
    failed_domains.contains(format!("supplier:{}", supply.supplier_id).as_str())
        || failed_domains.contains(format!("capacity-pool:{}", supply.capacity_pool_id).as_str())
        || supply.failure_domains.iter().any(|domain| failed_domains.contains(domain.as_str()))
}

fn checked_add(total: u64, value: u64) -> Result<u64, RecourseError> {
    total.checked_add(value).ok_or(RecourseError::ArithmeticOverflow)
}

fn checked_mul(left: u64, right: u64) -> Result<u64, RecourseError> {
    left.checked_mul(right).ok_or(RecourseError::ArithmeticOverflow)
}

#[derive(Clone)]
struct EligibleOption {
    option: ContingencyOption,
    activation_at: u64,
    arrival_at: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct RecoursePlan {
    allocations: Vec<RecourseAllocation>,
    recovered_quantity: u32,
    cost_minor: u64,
    latest_arrival_at: Option<u64>,
    flexible_quantity: u32,
    backup_quantity: u32,
}

struct ScenarioSearch<'a> {
    options: &'a [EligibleOption],
    target_quantity: u32,
    remaining_budget_minor: u64,
    max_nodes: u64,
    nodes: u64,
    best: Option<RecoursePlan>,
}

impl ScenarioSearch<'_> {
    fn recurse(
        &mut self,
        index: usize,
        quantities: &mut [u32],
    ) -> Result<(), RecourseError> {
        self.nodes = self.nodes.checked_add(1).ok_or(RecourseError::ArithmeticOverflow)?;
        if self.nodes > self.max_nodes {
            return Err(RecourseError::SearchSpaceExceeded);
        }

        let mut total_quantity = 0u64;
        let mut total_cost = 0u64;
        for (option, quantity) in self.options.iter().zip(quantities.iter()).take(index) {
            total_quantity = checked_add(total_quantity, u64::from(*quantity))?;
            if *quantity > 0 {
                let variable = checked_mul(option.option.unit_cost_minor, u64::from(*quantity))?;
                total_cost = checked_add(total_cost, variable)?;
                total_cost = checked_add(total_cost, option.option.activation_fee_minor)?;
            }
        }
        if total_quantity > u64::from(self.target_quantity)
            || total_cost > self.remaining_budget_minor
        {
            return Ok(());
        }

        if index == self.options.len() {
            let plan = self.build_plan(quantities)?;
            if self.best.as_ref().map_or(true, |best| plan_is_better(&plan, best)) {
                self.best = Some(plan);
            }
            return Ok(());
        }

        let option = &self.options[index].option;
        let max_quantity = option.max_activation_quantity.min(self.target_quantity);
        let choice_count = if option.minimum_order_quantity <= max_quantity {
            u64::from((max_quantity - option.minimum_order_quantity) / option.order_increment) + 1
        } else {
            0
        };
        if choice_count > MAX_RECOURSE_QUANTITY_CHOICES {
            return Err(RecourseError::SearchSpaceExceeded);
        }

        quantities[index] = 0;
        self.recurse(index + 1, quantities)?;

        if choice_count > 0 {
            let mut quantity = option.minimum_order_quantity;
            loop {
                quantities[index] = quantity;
                self.recurse(index + 1, quantities)?;
                let Some(next) = quantity.checked_add(option.order_increment) else {
                    break;
                };
                if next > max_quantity {
                    break;
                }
                quantity = next;
            }
        }
        quantities[index] = 0;
        Ok(())
    }

    fn build_plan(&self, quantities: &[u32]) -> Result<RecoursePlan, RecourseError> {
        let mut allocations = Vec::new();
        let mut recovered_quantity = 0u32;
        let mut cost_minor = 0u64;
        let mut latest_arrival_at = None;
        let mut flexible_quantity = 0u32;
        let mut backup_quantity = 0u32;

        for (eligible, quantity) in self.options.iter().zip(quantities.iter()) {
            if *quantity == 0 {
                continue;
            }
            recovered_quantity = recovered_quantity
                .checked_add(*quantity)
                .ok_or(RecourseError::ArithmeticOverflow)?;
            let variable = checked_mul(eligible.option.unit_cost_minor, u64::from(*quantity))?;
            let allocated_cost = checked_add(variable, eligible.option.activation_fee_minor)?;
            cost_minor = checked_add(cost_minor, allocated_cost)?;
            latest_arrival_at = Some(
                latest_arrival_at.map_or(eligible.arrival_at, |current: u64| current.max(eligible.arrival_at)),
            );
            match eligible.option.mode {
                SupplyMode::Flexible => {
                    flexible_quantity = flexible_quantity.checked_add(*quantity)
                        .ok_or(RecourseError::ArithmeticOverflow)?;
                }
                SupplyMode::Backup => {
                    backup_quantity = backup_quantity.checked_add(*quantity)
                        .ok_or(RecourseError::ArithmeticOverflow)?;
                }
            }
            allocations.push(RecourseAllocation {
                offer_id: eligible.option.supply.offer_id.clone(),
                supplier_id: eligible.option.supply.supplier_id.clone(),
                mode: eligible.option.mode,
                capacity_pool_id: eligible.option.supply.capacity_pool_id.clone(),
                capacity_snapshot_digest: eligible.option.supply.capacity_snapshot_digest.clone(),
                qualification_receipt_id: eligible.option.supply.qualification_receipt_id.clone(),
                source_uri: eligible.option.supply.source_uri.clone(),
                source_payload_digest: eligible.option.supply.source_payload_digest.clone(),
                quantity: *quantity,
                activation_at: eligible.activation_at,
                arrival_at: eligible.arrival_at,
                allocated_cost_minor: allocated_cost,
            });
        }
        allocations.sort_by(|left, right| left.offer_id.cmp(&right.offer_id));

        Ok(RecoursePlan {
            allocations,
            recovered_quantity,
            cost_minor,
            latest_arrival_at,
            flexible_quantity,
            backup_quantity,
        })
    }
}

/// Policy is explicitly lexicographic: maximize on-time recovery, then minimize
/// incremental recourse cost, then earliest latest-arrival, then stable offer IDs.
fn plan_is_better(candidate: &RecoursePlan, incumbent: &RecoursePlan) -> bool {
    candidate.recovered_quantity > incumbent.recovered_quantity
        || (candidate.recovered_quantity == incumbent.recovered_quantity
            && (candidate.cost_minor < incumbent.cost_minor
                || (candidate.cost_minor == incumbent.cost_minor
                    && (candidate.latest_arrival_at.unwrap_or(0) < incumbent.latest_arrival_at.unwrap_or(0)
                        || (candidate.latest_arrival_at == incumbent.latest_arrival_at
                            && candidate.allocations < incumbent.allocations)))))
}

fn allocation_choices(
    option: &ContingencyOption,
    lost_quantity: u32,
    max_choice_count: u64,
) -> Result<u64, RecourseError> {
    let max_quantity = option.max_activation_quantity.min(lost_quantity);
    let nonzero = if option.minimum_order_quantity <= max_quantity {
        u64::from((max_quantity - option.minimum_order_quantity) / option.order_increment) + 1
    } else {
        0
    };
    let choice_count = nonzero.checked_add(1).ok_or(RecourseError::ArithmeticOverflow)?;
    if choice_count > max_choice_count || choice_count > MAX_RECOURSE_QUANTITY_CHOICES {
        return Err(RecourseError::SearchSpaceExceeded);
    }
    Ok(choice_count)
}

fn option_validation_reason(
    request: &RecourseRequest,
    option: &ContingencyOption,
    scenario: &RecourseScenario,
) -> Option<SupplyRejection> {
    let evidence = &option.supply;
    if let Some(reason) = evidence_rejection(request, evidence, request.evaluated_at, true) {
        return Some(reason);
    }
    if let Some(reason) = evidence_rejection(request, evidence, scenario.observed_at, true) {
        return Some(reason);
    }
    if option.max_activation_quantity == 0
        || option.max_activation_quantity > evidence.capacity_pool_quantity
        || option.minimum_order_quantity == 0
        || option.order_increment == 0
        || option.minimum_order_quantity > option.max_activation_quantity
    {
        return Some(SupplyRejection::InvalidLotPolicy);
    }
    let activation_at = scenario.observed_at.checked_add(option.activation_lead_secs);
    let Some(activation_at) = activation_at else {
        return Some(SupplyRejection::TimeOverflow);
    };
    let arrival_at = activation_at.checked_add(option.transit_secs);
    let Some(arrival_at) = arrival_at else {
        return Some(SupplyRejection::TimeOverflow);
    };
    if arrival_at > request.deliver_by {
        return Some(SupplyRejection::ActivationAfterDeadline);
    }
    None
}

fn option_for_scenario(
    request: &RecourseRequest,
    option: &ContingencyOption,
    scenario: &RecourseScenario,
    failed_domains: &BTreeSet<&str>,
) -> Result<Result<EligibleOption, SupplyRejection>, RecourseError> {
    if let Some(reason) = option_validation_reason(request, option, scenario) {
        return Ok(Err(reason));
    }
    if failed(&option.supply, failed_domains) {
        return Ok(Err(SupplyRejection::FailedInScenario));
    }
    let activation_at = scenario.observed_at
        .checked_add(option.activation_lead_secs)
        .ok_or(RecourseError::ArithmeticOverflow)?;
    let arrival_at = activation_at
        .checked_add(option.transit_secs)
        .ok_or(RecourseError::ArithmeticOverflow)?;
    Ok(Ok(EligibleOption {
        option: option.clone(),
        activation_at,
        arrival_at,
    }))
}

fn baseline_and_reservation_cost(
    baseline: &[BaseAllocation],
    options: &[ContingencyOption],
) -> Result<u64, RecourseError> {
    let mut total = 0u64;
    for allocation in baseline {
        total = checked_add(
            total,
            checked_mul(u64::from(allocation.quantity), allocation.unit_cost_minor)?,
        )?;
        total = checked_add(total, allocation.fixed_order_cost_minor)?;
    }
    for option in options {
        total = checked_add(total, option.reservation_fee_minor)?;
    }
    Ok(total)
}

/// Evaluate fixed baseline commitments and pre-committed flexible/backup options
/// against a frozen scenario set. Each scenario's recourse allocation is solved
/// independently, within an explicit global node budget; it cannot alter the base
/// plan or authorize real-world changes.
pub fn evaluate_recourse(
    request: &RecourseRequest,
    baseline: &[BaseAllocation],
    options: &[ContingencyOption],
    scenarios: &[RecourseScenario],
) -> Result<RecourseReport, RecourseError> {
    request.validate(scenarios.len())?;
    if baseline.is_empty()
        || baseline.len() > MAX_BASE_ALLOCATIONS
        || options.len() > MAX_CONTINGENCY_OPTIONS
    {
        return Err(RecourseError::InvalidBaselinePlan);
    }

    let baseline_quantity = baseline.iter().try_fold(0u64, |total, allocation| {
        total.checked_add(u64::from(allocation.quantity))
            .ok_or(RecourseError::ArithmeticOverflow)
    })?;
    if baseline_quantity != u64::from(request.required_quantity) {
        return Err(RecourseError::InvalidBaselinePlan);
    }

    let mut scenario_ids = BTreeSet::new();
    for scenario in scenarios {
        if !non_empty(&scenario.scenario_id)
            || scenario.observed_at < request.evaluated_at
            || scenario.observed_at > request.deliver_by
            || scenario.failed_domains.iter().any(|domain| !non_empty(domain))
        {
            return Err(RecourseError::InvalidScenario);
        }
        if !scenario_ids.insert(scenario.scenario_id.as_str()) {
            return Err(RecourseError::DuplicateScenarioId);
        }
    }

    let mut offer_ids = BTreeSet::new();
    let mut pool_snapshots: BTreeMap<String, CapacityPoolSnapshot> = BTreeMap::new();
    let mut pool_reserved: BTreeMap<String, u64> = BTreeMap::new();

    for allocation in baseline {
        let supply = &allocation.supply;
        if !offer_ids.insert(supply.offer_id.as_str()) {
            return Err(RecourseError::DuplicateOfferId);
        }
        if evidence_rejection(request, supply, request.evaluated_at, true).is_some()
            || !validate_base_lot(allocation)
            || allocation.arrival_at > request.deliver_by
        {
            return Err(RecourseError::InvalidBaselineEvidence);
        }
        let snapshot = capacity_pool_snapshot(supply);
        if let Some(existing) = pool_snapshots.get(&supply.capacity_pool_id) {
            if existing != &snapshot {
                return Err(RecourseError::InconsistentCapacityPool);
            }
        } else {
            pool_snapshots.insert(supply.capacity_pool_id.clone(), snapshot);
        }
        let reserved = pool_reserved.entry(supply.capacity_pool_id.clone()).or_default();
        *reserved = checked_add(*reserved, u64::from(allocation.quantity))?;
    }

    for option in options {
        let supply = &option.supply;
        if !offer_ids.insert(supply.offer_id.as_str()) {
            return Err(RecourseError::DuplicateOfferId);
        }
        if !non_empty(&supply.capacity_pool_id)
            || !non_empty(&supply.capacity_snapshot_digest)
            || !non_empty(&supply.offer_id)
        {
            return Err(RecourseError::InvalidBaselineEvidence);
        }
        let snapshot = capacity_pool_snapshot(supply);
        if let Some(existing) = pool_snapshots.get(&supply.capacity_pool_id) {
            if existing != &snapshot {
                return Err(RecourseError::InconsistentCapacityPool);
            }
        } else {
            pool_snapshots.insert(supply.capacity_pool_id.clone(), snapshot);
        }
        let reserved = pool_reserved.entry(supply.capacity_pool_id.clone()).or_default();
        *reserved = checked_add(*reserved, u64::from(option.max_activation_quantity))?;
    }

    for (pool_id, reserved) in &pool_reserved {
        let cap = pool_snapshots.get(pool_id).ok_or(RecourseError::InconsistentCapacityPool)?.quantity;
        if *reserved > u64::from(cap) {
            return Err(RecourseError::CapacityPoolOvercommitted);
        }
    }

    let baseline_cost = baseline_and_reservation_cost(baseline, options)?;
    if baseline_cost > request.max_total_cost_minor {
        return Err(RecourseError::BaseCommitmentOverBudget);
    }

    let mut sorted_baseline = baseline.to_vec();
    sorted_baseline.sort_by(|left, right| left.supply.offer_id.cmp(&right.supply.offer_id));
    let mut sorted_options = options.to_vec();
    sorted_options.sort_by(|left, right| left.supply.offer_id.cmp(&right.supply.offer_id));
    let mut sorted_scenarios = scenarios.to_vec();
    for scenario in &mut sorted_scenarios {
        scenario.failed_domains.sort();
        scenario.failed_domains.dedup();
    }
    sorted_scenarios.sort_by(|left, right| left.scenario_id.cmp(&right.scenario_id));

    let scenario_budget = request.max_search_nodes / sorted_scenarios.len() as u64;
    if scenario_budget == 0 {
        return Err(RecourseError::SearchSpaceExceeded);
    }
    let mut total_search_nodes = 0u64;
    let mut scenario_results = Vec::with_capacity(sorted_scenarios.len());

    for scenario in &sorted_scenarios {
        let failed_domains: BTreeSet<&str> =
            scenario.failed_domains.iter().map(String::as_str).collect();
        let mut base_lost_quantity = 0u32;
        for allocation in &sorted_baseline {
            if failed(&allocation.supply, &failed_domains) {
                base_lost_quantity = base_lost_quantity
                    .checked_add(allocation.quantity)
                    .ok_or(RecourseError::ArithmeticOverflow)?;
            }
        }
        let surviving_base_quantity = request.required_quantity
            .checked_sub(base_lost_quantity)
            .ok_or(RecourseError::ArithmeticOverflow)?;

        let mut excluded_options = Vec::new();
        let mut eligible_options = Vec::new();
        if base_lost_quantity > 0 {
            for option in &sorted_options {
                match option_for_scenario(request, option, scenario, &failed_domains)? {
                    Ok(eligible) => eligible_options.push(eligible),
                    Err(reason) => excluded_options.push(ExcludedOption {
                        offer_id: option.supply.offer_id.clone(),
                        supplier_id: option.supply.supplier_id.clone(),
                        reason,
                    }),
                }
            }
        }
        excluded_options.sort_by(|left, right| left.offer_id.cmp(&right.offer_id));
        eligible_options.sort_by(|left, right| left.option.supply.offer_id.cmp(&right.option.supply.offer_id));

        let mut best = RecoursePlan {
            allocations: Vec::new(),
            recovered_quantity: 0,
            cost_minor: 0,
            latest_arrival_at: None,
            flexible_quantity: 0,
            backup_quantity: 0,
        };
        let mut scenario_search_nodes = 0u64;
        if base_lost_quantity > 0 {
            for eligible in &eligible_options {
                allocation_choices(
                    &eligible.option,
                    base_lost_quantity,
                    scenario_budget,
                )?;
            }
            let mut search = ScenarioSearch {
                options: &eligible_options,
                target_quantity: base_lost_quantity,
                remaining_budget_minor: request.max_total_cost_minor - baseline_cost,
                max_nodes: scenario_budget,
                nodes: 0,
                best: None,
            };
            let mut quantities = vec![0u32; eligible_options.len()];
            search.recurse(0, &mut quantities)?;
            scenario_search_nodes = search.nodes;
            if let Some(plan) = search.best {
                best = plan;
            }
        }

        total_search_nodes = total_search_nodes
            .checked_add(scenario_search_nodes)
            .ok_or(RecourseError::ArithmeticOverflow)?;
        let unfilled_quantity = base_lost_quantity
            .checked_sub(best.recovered_quantity)
            .ok_or(RecourseError::ArithmeticOverflow)?;
        let total_scenario_cost_minor = checked_add(baseline_cost, best.cost_minor)?;
        scenario_results.push(RecourseScenarioResult {
            scenario_id: scenario.scenario_id.clone(),
            observed_at: scenario.observed_at,
            failed_domains: scenario.failed_domains.clone(),
            base_lost_quantity,
            surviving_base_quantity,
            recovered_quantity: best.recovered_quantity,
            unfilled_quantity,
            recourse_cost_minor: best.cost_minor,
            total_scenario_cost_minor,
            latest_recovery_arrival_at: best.latest_arrival_at,
            flexible_quantity: best.flexible_quantity,
            backup_quantity: best.backup_quantity,
            allocations: best.allocations,
            excluded_options,
            search_nodes: scenario_search_nodes,
        });
    }

    let contingency_option_ids = sorted_options
        .iter()
        .map(|option| option.supply.offer_id.clone())
        .collect();

    Ok(RecourseReport {
        evaluator_profile: EVALUATOR_PROFILE,
        decision_policy: DECISION_POLICY,
        request_id: request.request_id.clone(),
        evaluated_at: request.evaluated_at,
        capability_id: request.capability_id.clone(),
        capability_profile_digest: request.capability_profile_digest.clone(),
        unit: request.unit.clone(),
        currency: request.currency.clone(),
        required_quantity: request.required_quantity,
        policy_id: request.policy_id.clone(),
        policy_digest: request.policy_digest.clone(),
        scenario_set_id: request.scenario_set_id.clone(),
        scenario_set_digest: request.scenario_set_digest.clone(),
        input_manifest_digest: request.input_manifest_digest.clone(),
        baseline: sorted_baseline,
        contingency_option_ids,
        baseline_and_reservation_cost_minor: baseline_cost,
        scenario_results,
        total_search_nodes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request() -> RecourseRequest {
        RecourseRequest {
            request_id: "request-1".into(),
            capability_id: "capability:fastener".into(),
            capability_profile_digest: "sha256:profile-1".into(),
            unit: "each".into(),
            currency: "USD".into(),
            required_evidence_origin: EvidenceOrigin::SyntheticSimulation,
            evaluated_at: 100,
            required_quantity: 10,
            deliver_by: 130,
            max_total_cost_minor: 200,
            max_qualification_age_secs: 50,
            max_quote_age_secs: 30,
            max_capacity_age_secs: 30,
            max_dependency_age_secs: 50,
            policy_id: "policy:test-v1".into(),
            policy_digest: "sha256:policy-v1".into(),
            scenario_set_id: "scenarios:test-v1".into(),
            scenario_set_digest: "sha256:scenarios-v1".into(),
            input_manifest_digest: "sha256:manifest-v1".into(),
            max_search_nodes: 50_000,
        }
    }

    fn supply(id: &str, supplier: &str, pool: &str, capacity: u32, domain: &str) -> SupplyEvidence {
        SupplyEvidence {
            offer_id: id.into(),
            supplier_id: supplier.into(),
            source_uri: format!("https://fixture.invalid/{id}"),
            source_payload_digest: format!("sha256:source-{id}"),
            extraction_profile_id: "extractor:fixture-v1".into(),
            qualification_receipt_id: format!("qualification:{id}"),
            capability_id: "capability:fastener".into(),
            capability_profile_digest: "sha256:profile-1".into(),
            unit: "each".into(),
            currency: "USD".into(),
            qualification_status: EvidenceStatus::Verified,
            qualification_origin: EvidenceOrigin::SyntheticSimulation,
            qualification_observed_at: 95,
            qualification_valid_until: 150,
            quote_status: EvidenceStatus::Verified,
            quote_origin: EvidenceOrigin::SyntheticSimulation,
            quote_observed_at: 96,
            quote_valid_until: 145,
            capacity_status: EvidenceStatus::Verified,
            capacity_origin: EvidenceOrigin::SyntheticSimulation,
            capacity_pool_id: pool.into(),
            capacity_snapshot_digest: format!("sha256:capacity-{pool}"),
            capacity_pool_quantity: capacity,
            capacity_observed_at: 97,
            capacity_valid_until: 145,
            failure_domains: vec![domain.into()],
            failure_domain_status: EvidenceStatus::Verified,
            failure_domain_origin: EvidenceOrigin::SyntheticSimulation,
            failure_domain_observed_at: 95,
            failure_domain_valid_until: 150,
        }
    }

    fn base(id: &str, supplier: &str, pool: &str, quantity: u32, capacity: u32, domain: &str) -> BaseAllocation {
        BaseAllocation {
            supply: supply(id, supplier, pool, capacity, domain),
            quantity,
            max_allocatable_quantity: capacity,
            minimum_order_quantity: 1,
            order_increment: 1,
            unit_cost_minor: 1,
            fixed_order_cost_minor: 0,
            arrival_at: 120,
        }
    }

    fn option(
        id: &str,
        supplier: &str,
        pool: &str,
        capacity: u32,
        domain: &str,
        mode: SupplyMode,
        unit_cost: u64,
        activation_fee: u64,
        reservation_fee: u64,
        activation_lead: u64,
        transit: u64,
    ) -> ContingencyOption {
        ContingencyOption {
            supply: supply(id, supplier, pool, capacity, domain),
            mode,
            max_activation_quantity: capacity,
            minimum_order_quantity: 1,
            order_increment: 1,
            activation_lead_secs: activation_lead,
            transit_secs: transit,
            unit_cost_minor: unit_cost,
            activation_fee_minor: activation_fee,
            reservation_fee_minor: reservation_fee,
        }
    }

    #[test]
    fn flexible_then_backup_recovers_disrupted_baseline() {
        let req = request();
        let baseline = vec![base("base-a", "supplier-a", "pool-base", 10, 10, "region-a")];
        let options = vec![
            option("flex-b", "supplier-b", "pool-flex", 4, "region-b", SupplyMode::Flexible, 2, 1, 2, 2, 3),
            option("backup-c", "supplier-c", "pool-backup", 8, "region-c", SupplyMode::Backup, 4, 2, 3, 5, 5),
        ];
        let scenarios = vec![
            RecourseScenario { scenario_id: "no-disruption".into(), observed_at: 105, failed_domains: vec![] },
            RecourseScenario { scenario_id: "supplier-a-fails".into(), observed_at: 105, failed_domains: vec!["supplier:supplier-a".into()] },
        ];
        let report = evaluate_recourse(&req, &baseline, &options, &scenarios).unwrap();
        assert_eq!(report.baseline_and_reservation_cost_minor, 15);
        let control = report.scenario_results.iter().find(|s| s.scenario_id == "no-disruption").unwrap();
        assert_eq!(control.base_lost_quantity, 0);
        assert_eq!(control.recovered_quantity, 0);
        assert_eq!(control.total_scenario_cost_minor, 15);
        let disrupted = report.scenario_results.iter().find(|s| s.scenario_id == "supplier-a-fails").unwrap();
        assert_eq!(disrupted.base_lost_quantity, 10);
        assert_eq!(disrupted.recovered_quantity, 10);
        assert_eq!(disrupted.unfilled_quantity, 0);
        assert_eq!(disrupted.flexible_quantity, 4);
        assert_eq!(disrupted.backup_quantity, 6);
        assert_eq!(disrupted.latest_recovery_arrival_at, Some(115));
        assert_eq!(disrupted.total_scenario_cost_minor, 50);
    }

    #[test]
    fn mixed_flexible_backup_choice_matches_two_option_exhaustive_oracle() {
        let req = request();
        let baseline = vec![
            base("base-a", "supplier-a", "pool-base-a", 3, 10, "region-a"),
            base("base-b", "supplier-b", "pool-base-b", 7, 10, "region-b"),
        ];
        let options = vec![
            option("flex-c", "supplier-c", "pool-flex-c", 2, "region-c", SupplyMode::Flexible, 2, 1, 2, 2, 3),
            option("backup-d", "supplier-d", "pool-backup-d", 3, "region-d", SupplyMode::Backup, 4, 2, 3, 5, 5),
        ];
        let scenarios = vec![RecourseScenario {
            scenario_id: "supplier-a-fails".into(),
            observed_at: 105,
            failed_domains: vec!["supplier:supplier-a".into()],
        }];

        // Separate exhaustive oracle over every pair of allowed activation quantities.
        // It does not call the production search, dominance or selection functions.
        let mut expected_recovered = 0u32;
        let mut expected_cost = 0u64;
        let mut expected_latest = 0u64;
        for flexible_quantity in 0..=2u32 {
            for backup_quantity in 0..=3u32 {
                let recovered = flexible_quantity + backup_quantity;
                if recovered > 3 {
                    continue;
                }
                let flex_cost = if flexible_quantity == 0 {
                    0
                } else {
                    u64::from(flexible_quantity) * 2 + 1
                };
                let backup_cost = if backup_quantity == 0 {
                    0
                } else {
                    u64::from(backup_quantity) * 4 + 2
                };
                let cost = flex_cost + backup_cost;
                let latest = if backup_quantity > 0 {
                    115
                } else if flexible_quantity > 0 {
                    110
                } else {
                    0
                };
                let is_better = recovered > expected_recovered
                    || (recovered == expected_recovered
                        && (cost < expected_cost
                            || (cost == expected_cost && latest < expected_latest)));
                if is_better {
                    expected_recovered = recovered;
                    expected_cost = cost;
                    expected_latest = latest;
                }
            }
        }

        let report = evaluate_recourse(&req, &baseline, &options, &scenarios).unwrap();
        let result = &report.scenario_results[0];
        assert_eq!(result.recovered_quantity, expected_recovered);
        assert_eq!(result.recourse_cost_minor, expected_cost);
        assert_eq!(result.latest_recovery_arrival_at, Some(expected_latest));
        assert_eq!(result.flexible_quantity, 2);
        assert_eq!(result.backup_quantity, 1);
        assert_eq!(expected_cost, 11);
    }

    #[test]
    fn backup_that_arrives_after_deadline_is_not_counted_as_recovery() {
        let req = request();
        let baseline = vec![base("base-a", "supplier-a", "pool-base", 10, 10, "region-a")];
        let options = vec![
            option("backup-b", "supplier-b", "pool-backup", 10, "region-b", SupplyMode::Backup, 2, 1, 2, 20, 20),
        ];
        let scenarios = vec![RecourseScenario {
            scenario_id: "supplier-a-fails".into(),
            observed_at: 105,
            failed_domains: vec!["supplier:supplier-a".into()],
        }];
        let report = evaluate_recourse(&req, &baseline, &options, &scenarios).unwrap();
        let result = &report.scenario_results[0];
        assert_eq!(result.recovered_quantity, 0);
        assert_eq!(result.unfilled_quantity, 10);
        assert_eq!(result.excluded_options[0].reason, SupplyRejection::ActivationAfterDeadline);
    }

    #[test]
    fn search_budget_exhaustion_returns_error_not_partial_result() {
        let mut req = request();
        req.max_search_nodes = 4;
        let baseline = vec![base("base-a", "supplier-a", "pool-base", 10, 10, "region-a")];
        let contingency = option(
            "backup-b", "supplier-b", "pool-backup", 10, "region-b",
            SupplyMode::Backup, 2, 1, 2, 2, 3,
        );
        let result = evaluate_recourse(&req, &baseline, &[contingency], &[RecourseScenario {
            scenario_id: "supplier-a-fails".into(),
            observed_at: 105,
            failed_domains: vec!["supplier:supplier-a".into()],
        }]);
        assert_eq!(result, Err(RecourseError::SearchSpaceExceeded));
    }

    #[test]
    fn expired_backup_quote_cannot_be_promoted_after_disruption() {
        let req = request();
        let baseline = vec![base("base-a", "supplier-a", "pool-base", 10, 10, "region-a")];
        let mut backup = option("backup-b", "supplier-b", "pool-backup", 10, "region-b", SupplyMode::Backup, 2, 1, 2, 2, 3);
        backup.supply.quote_valid_until = 103;
        let scenarios = vec![RecourseScenario {
            scenario_id: "supplier-a-fails".into(),
            observed_at: 105,
            failed_domains: vec!["supplier:supplier-a".into()],
        }];
        let report = evaluate_recourse(&req, &baseline, &[backup], &scenarios).unwrap();
        assert_eq!(report.scenario_results[0].recovered_quantity, 0);
        assert_eq!(report.scenario_results[0].excluded_options[0].reason, SupplyRejection::QuoteExpired);
    }

    #[test]
    fn recourse_quantity_and_cost_match_small_exhaustive_oracle() {
        let req = request();
        let baseline = vec![
            base("base-a", "supplier-a", "pool-base-a", 6, 10, "region-a"),
            base("base-b", "supplier-b", "pool-base-b", 4, 10, "region-b"),
        ];
        let contingency = option(
            "backup-c", "supplier-c", "pool-backup-c", 6, "region-c",
            SupplyMode::Backup, 2, 1, 2, 2, 3,
        );
        let scenarios = vec![RecourseScenario {
            scenario_id: "supplier-a-fails".into(),
            observed_at: 105,
            failed_domains: vec!["supplier:supplier-a".into()],
        }];

        // Independent tiny oracle: enumerate every allowable recourse quantity
        // for the one backup offer, maximize recovery, then minimize incremental cost.
        let lost = 6u32;
        let mut expected_quantity = 0u32;
        let mut expected_cost = 0u64;
        for quantity in 0..=lost {
            let cost = if quantity == 0 { 0 } else { u64::from(quantity) * 2 + 1 };
            if cost <= req.max_total_cost_minor - 12
                && (quantity > expected_quantity
                    || (quantity == expected_quantity && cost < expected_cost))
            {
                expected_quantity = quantity;
                expected_cost = cost;
            }
        }

        let report = evaluate_recourse(&req, &baseline, &[contingency], &scenarios).unwrap();
        let result = &report.scenario_results[0];
        assert_eq!(result.recovered_quantity, expected_quantity);
        assert_eq!(result.recourse_cost_minor, expected_cost);
        assert_eq!(result.unfilled_quantity, lost - expected_quantity);
        assert_eq!(expected_quantity, 6);
        assert_eq!(expected_cost, 13);
    }

    #[test]
    fn baseline_must_conserve_requested_quantity() {
        let req = request();
        let incomplete = vec![base("base-a", "supplier-a", "pool-base", 9, 10, "region-a")];
        let result = evaluate_recourse(&req, &incomplete, &[], &[RecourseScenario {
            scenario_id: "no-disruption".into(),
            observed_at: 105,
            failed_domains: vec![],
        }]);
        assert_eq!(result, Err(RecourseError::InvalidBaselinePlan));
    }

    #[test]
    fn shared_capacity_pool_reservation_cannot_be_double_counted() {
        let req = request();
        let baseline = vec![base("base-a", "supplier-a", "shared", 10, 10, "region-a")];
        let options = vec![option("backup-b", "supplier-b", "shared", 10, "region-b", SupplyMode::Backup, 2, 1, 2, 2, 3)];
        assert_eq!(
            evaluate_recourse(&req, &baseline, &options, &[RecourseScenario {
                scenario_id: "supplier-a-fails".into(),
                observed_at: 105,
                failed_domains: vec!["supplier:supplier-a".into()],
            }]),
            Err(RecourseError::CapacityPoolOvercommitted)
        );
    }

    #[test]
    fn contingency_on_shared_failure_domain_is_not_independent_recovery() {
        let req = request();
        let baseline = vec![base("base-a", "supplier-a", "pool-base", 10, 10, "region-common")];
        let options = vec![option("backup-b", "supplier-b", "pool-backup", 10, "region-common", SupplyMode::Backup, 2, 1, 2, 2, 3)];
        let report = evaluate_recourse(&req, &baseline, &options, &[RecourseScenario {
            scenario_id: "region-fails".into(),
            observed_at: 105,
            failed_domains: vec!["region-common".into()],
        }]).unwrap();
        assert_eq!(report.scenario_results[0].base_lost_quantity, 10);
        assert_eq!(report.scenario_results[0].recovered_quantity, 0);
        assert_eq!(report.scenario_results[0].unfilled_quantity, 10);
    }

    #[test]
    fn scenario_and_candidate_order_do_not_change_canonical_report() {
        let req = request();
        let baseline = vec![base("base-a", "supplier-a", "pool-base", 10, 10, "region-a")];
        let a = option("backup-b", "supplier-b", "pool-b", 10, "region-b", SupplyMode::Backup, 2, 1, 2, 2, 3);
        let b = option("flex-c", "supplier-c", "pool-c", 10, "region-c", SupplyMode::Flexible, 3, 2, 1, 1, 4);
        let scenarios = vec![
            RecourseScenario { scenario_id: "z-disruption".into(), observed_at: 105, failed_domains: vec!["supplier:supplier-a".into()] },
            RecourseScenario { scenario_id: "a-no-disruption".into(), observed_at: 105, failed_domains: vec![] },
        ];
        let left = evaluate_recourse(&req, &baseline, &[a.clone(), b.clone()], &scenarios).unwrap();
        let right = evaluate_recourse(&req, &baseline, &[b, a], &scenarios.into_iter().rev().collect::<Vec<_>>()).unwrap();
        assert_eq!(left, right);
    }
}
