//! Deterministic model of hierarchical retry and hedge amplification budgets.
//! Eligibility is supplied by a separately qualified operation-semantics
//! contract; budget capacity can never make an ineligible retry eligible.
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RetryMode {
    SequentialRetry,
    Hedge,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RetryEligibility {
    Denied,
    SequentialOnly,
    SequentialAndHedge,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct BudgetPolicy {
    global_ratio_numerator: u64,
    global_ratio_denominator: u64,
    global_minimum_extra: u64,
    workload_ratio_numerator: u64,
    workload_ratio_denominator: u64,
    workload_minimum_extra: u64,
    max_global_extra_in_flight: u64,
    max_workload_extra_in_flight: u64,
    max_hedges_in_flight: u64,
    max_extra_attempts_per_operation: u32,
}

impl BudgetPolicy {
    fn validate(self) -> Result<Self, BudgetError> {
        if self.global_ratio_denominator == 0
            || self.workload_ratio_denominator == 0
            || self.max_global_extra_in_flight == 0
            || self.max_workload_extra_in_flight == 0
            || self.max_extra_attempts_per_operation == 0
        {
            return Err(BudgetError::InvalidPolicy);
        }
        Ok(self)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BudgetError {
    InvalidPolicy,
    InvalidIdentity,
    OperationAlreadyActive,
    OperationNotActive,
    WorkloadMismatch,
    SemanticsNotEligible,
    OperationExtraLimit,
    GlobalInflightLimit,
    WorkloadInflightLimit,
    HedgeInflightLimit,
    GlobalRatioLimit,
    WorkloadRatioLimit,
    OperationHasInflightExtra,
    UnknownPermit,
    CounterOverflow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ExtraPermit {
    permit_id: u64,
    operation_id: u64,
    workload_id: u64,
    mode: RetryMode,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PermitRecord {
    permit: ExtraPermit,
}

#[derive(Clone, Debug)]
struct RetryBudget {
    policy: BudgetPolicy,
    active_operations: HashMap<u64, u64>,
    permits: HashMap<u64, PermitRecord>,
    total_extra_attempts_by_operation: HashMap<u64, u32>,
    next_permit_id: u64,
}

impl RetryBudget {
    fn new(policy: BudgetPolicy) -> Result<Self, BudgetError> {
        Ok(Self {
            policy: policy.validate()?,
            active_operations: HashMap::new(),
            permits: HashMap::new(),
            total_extra_attempts_by_operation: HashMap::new(),
            next_permit_id: 1,
        })
    }

    fn begin_operation(
        &mut self,
        operation_id: u64,
        workload_id: u64,
    ) -> Result<(), BudgetError> {
        if operation_id == 0 || workload_id == 0 {
            return Err(BudgetError::InvalidIdentity);
        }
        if self.active_operations.contains_key(&operation_id) {
            return Err(BudgetError::OperationAlreadyActive);
        }
        self.active_operations.insert(operation_id, workload_id);
        Ok(())
    }

    fn finish_operation(&mut self, operation_id: u64) -> Result<(), BudgetError> {
        if !self.active_operations.contains_key(&operation_id) {
            return Err(BudgetError::OperationNotActive);
        }
        if self.permits.values().any(|record| {
            record.permit.operation_id == operation_id
        }) {
            return Err(BudgetError::OperationHasInflightExtra);
        }
        self.active_operations.remove(&operation_id);
        self.total_extra_attempts_by_operation.remove(&operation_id);
        Ok(())
    }

    fn reserve_extra(
        &mut self,
        operation_id: u64,
        workload_id: u64,
        mode: RetryMode,
        eligibility: RetryEligibility,
    ) -> Result<ExtraPermit, BudgetError> {
        match (mode, eligibility) {
            (_, RetryEligibility::Denied)
            | (RetryMode::Hedge, RetryEligibility::SequentialOnly) => {
                return Err(BudgetError::SemanticsNotEligible);
            }
            _ => {}
        }

        match self.active_operations.get(&operation_id) {
            None => return Err(BudgetError::OperationNotActive),
            Some(actual) if *actual != workload_id => {
                return Err(BudgetError::WorkloadMismatch);
            }
            Some(_) => {}
        }

        let operation_extra_total = self
            .total_extra_attempts_by_operation
            .get(&operation_id)
            .copied()
            .unwrap_or(0);
        if operation_extra_total >= self.policy.max_extra_attempts_per_operation {
            return Err(BudgetError::OperationExtraLimit);
        }

        let global_extra_active = self.permits.len() as u64;
        if global_extra_active >= self.policy.max_global_extra_in_flight {
            return Err(BudgetError::GlobalInflightLimit);
        }

        let workload_original_active = self
            .active_operations
            .values()
            .filter(|active_workload| **active_workload == workload_id)
            .count() as u64;
        let workload_extra_active = self
            .permits
            .values()
            .filter(|record| record.permit.workload_id == workload_id)
            .count() as u64;
        if workload_extra_active >= self.policy.max_workload_extra_in_flight {
            return Err(BudgetError::WorkloadInflightLimit);
        }

        if mode == RetryMode::Hedge {
            let active_hedges = self
                .permits
                .values()
                .filter(|record| record.permit.mode == RetryMode::Hedge)
                .count() as u64;
            if active_hedges >= self.policy.max_hedges_in_flight {
                return Err(BudgetError::HedgeInflightLimit);
            }
        }

        let global_original_active = self.active_operations.len() as u64;
        let global_ratio_cap = ratio_cap(
            global_original_active,
            self.policy.global_ratio_numerator,
            self.policy.global_ratio_denominator,
            self.policy.global_minimum_extra,
        );
        if global_extra_active + 1 > global_ratio_cap {
            return Err(BudgetError::GlobalRatioLimit);
        }

        let workload_ratio_cap = ratio_cap(
            workload_original_active,
            self.policy.workload_ratio_numerator,
            self.policy.workload_ratio_denominator,
            self.policy.workload_minimum_extra,
        );
        if workload_extra_active + 1 > workload_ratio_cap {
            return Err(BudgetError::WorkloadRatioLimit);
        }

        let permit_id = self.next_permit_id;
        self.next_permit_id = self
            .next_permit_id
            .checked_add(1)
            .ok_or(BudgetError::CounterOverflow)?;
        let next_total = operation_extra_total
            .checked_add(1)
            .ok_or(BudgetError::CounterOverflow)?;
        let permit = ExtraPermit {
            permit_id,
            operation_id,
            workload_id,
            mode,
        };
        self.total_extra_attempts_by_operation
            .insert(operation_id, next_total);
        self.permits.insert(permit_id, PermitRecord { permit });
        Ok(permit)
    }

    fn complete_extra(&mut self, permit: &ExtraPermit) -> Result<(), BudgetError> {
        match self.permits.get(&permit.permit_id) {
            Some(record) if record.permit == *permit => {
                self.permits.remove(&permit.permit_id);
                Ok(())
            }
            _ => Err(BudgetError::UnknownPermit),
        }
    }

    fn active_operation_count(&self) -> usize {
        self.active_operations.len()
    }

    fn active_extra_count(&self) -> usize {
        self.permits.len()
    }

    fn active_hedge_count(&self) -> usize {
        self.permits.values()
            .filter(|record| record.permit.mode == RetryMode::Hedge)
            .count()
    }

    fn operation_extra_count(&self, operation_id: u64) -> u32 {
        self.total_extra_attempts_by_operation
            .get(&operation_id)
            .copied()
            .unwrap_or(0)
    }
}

fn ratio_cap(
    active_operations: u64,
    numerator: u64,
    denominator: u64,
    minimum: u64,
) -> u64 {
    let ratio = ((active_operations as u128) * (numerator as u128))
        / (denominator as u128);
    ratio.max(minimum as u128).min(u64::MAX as u128) as u64
}

fn policy() -> BudgetPolicy {
    BudgetPolicy {
        global_ratio_numerator: 1,
        global_ratio_denominator: 1,
        global_minimum_extra: 1,
        workload_ratio_numerator: 1,
        workload_ratio_denominator: 1,
        workload_minimum_extra: 1,
        max_global_extra_in_flight: 16,
        max_workload_extra_in_flight: 4,
        max_hedges_in_flight: 4,
        max_extra_attempts_per_operation: 2,
    }
}

fn begin(budget: &mut RetryBudget, operation_id: u64, workload_id: u64) {
    budget.begin_operation(operation_id, workload_id).unwrap();
}

fn retry(budget: &mut RetryBudget, operation_id: u64, workload_id: u64) -> ExtraPermit {
    budget.reserve_extra(
        operation_id,
        workload_id,
        RetryMode::SequentialRetry,
        RetryEligibility::SequentialAndHedge,
    ).unwrap()
}

#[test]
fn semantics_ineligible_operation_cannot_spend_retry_budget() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    begin(&mut budget, 1, 7);
    assert_eq!(
        budget.reserve_extra(
            1, 7, RetryMode::SequentialRetry, RetryEligibility::Denied
        ),
        Err(BudgetError::SemanticsNotEligible)
    );
    assert_eq!(budget.active_extra_count(), 0);
}

#[test]
fn sequential_only_semantics_cannot_spend_hedge_budget() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    begin(&mut budget, 1, 7);
    assert_eq!(
        budget.reserve_extra(
            1, 7, RetryMode::Hedge, RetryEligibility::SequentialOnly
        ),
        Err(BudgetError::SemanticsNotEligible)
    );
}

#[test]
fn eligible_sequential_retry_reserves_one_extra_attempt() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    begin(&mut budget, 1, 7);
    let permit = retry(&mut budget, 1, 7);
    assert_eq!(budget.active_extra_count(), 1);
    assert_eq!(budget.operation_extra_count(1), 1);
    budget.complete_extra(&permit).unwrap();
    assert_eq!(budget.active_extra_count(), 0);
    assert_eq!(budget.operation_extra_count(1), 1);
}

#[test]
fn parallel_safe_semantics_can_reserve_a_hedge() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    begin(&mut budget, 1, 7);
    let permit = budget.reserve_extra(
        1, 7, RetryMode::Hedge, RetryEligibility::SequentialAndHedge
    ).unwrap();
    assert_eq!(budget.active_hedge_count(), 1);
    budget.complete_extra(&permit).unwrap();
    assert_eq!(budget.active_hedge_count(), 0);
}

#[test]
fn global_concurrency_cap_blocks_additional_extra_attempts() {
    let mut p = policy();
    p.max_global_extra_in_flight = 1;
    p.global_ratio_numerator = 10;
    p.workload_ratio_numerator = 10;
    let mut budget = RetryBudget::new(p).unwrap();
    begin(&mut budget, 1, 7);
    begin(&mut budget, 2, 8);
    let first = retry(&mut budget, 1, 7);
    assert_eq!(
        budget.reserve_extra(
            2, 8, RetryMode::SequentialRetry, RetryEligibility::SequentialAndHedge
        ),
        Err(BudgetError::GlobalInflightLimit)
    );
    budget.complete_extra(&first).unwrap();
}

#[test]
fn workload_concurrency_cap_blocks_one_workload_without_blocking_another() {
    let mut p = policy();
    p.max_workload_extra_in_flight = 1;
    p.max_global_extra_in_flight = 10;
    p.global_ratio_numerator = 10;
    p.workload_ratio_numerator = 10;
    let mut budget = RetryBudget::new(p).unwrap();
    begin(&mut budget, 1, 7);
    begin(&mut budget, 2, 7);
    begin(&mut budget, 3, 8);
    let first = retry(&mut budget, 1, 7);
    assert_eq!(
        budget.reserve_extra(
            2, 7, RetryMode::SequentialRetry, RetryEligibility::SequentialAndHedge
        ),
        Err(BudgetError::WorkloadInflightLimit)
    );
    let other_workload = retry(&mut budget, 3, 8);
    assert_eq!(budget.active_extra_count(), 2);
    budget.complete_extra(&first).unwrap();
    budget.complete_extra(&other_workload).unwrap();
}

#[test]
fn global_retry_ratio_is_enforced_against_active_operations() {
    let mut p = policy();
    p.global_ratio_numerator = 1;
    p.global_ratio_denominator = 2;
    p.global_minimum_extra = 0;
    p.workload_ratio_numerator = 10;
    let mut budget = RetryBudget::new(p).unwrap();
    begin(&mut budget, 1, 7);
    begin(&mut budget, 2, 8);
    begin(&mut budget, 3, 9);
    let a = retry(&mut budget, 1, 7);
    assert_eq!(
        budget.reserve_extra(
            2, 8, RetryMode::SequentialRetry, RetryEligibility::SequentialAndHedge
        ),
        Err(BudgetError::GlobalRatioLimit)
    );
    budget.complete_extra(&a).unwrap();
}

#[test]
fn workload_retry_ratio_is_enforced_independently() {
    let mut p = policy();
    p.global_ratio_numerator = 10;
    p.workload_ratio_numerator = 1;
    p.workload_ratio_denominator = 2;
    p.workload_minimum_extra = 0;
    let mut budget = RetryBudget::new(p).unwrap();
    begin(&mut budget, 1, 7);
    begin(&mut budget, 2, 7);
    begin(&mut budget, 3, 8);
    let a = retry(&mut budget, 1, 7);
    assert_eq!(
        budget.reserve_extra(
            2, 7, RetryMode::SequentialRetry, RetryEligibility::SequentialAndHedge
        ),
        Err(BudgetError::WorkloadRatioLimit)
    );
    budget.complete_extra(&a).unwrap();
}

#[test]
fn per_operation_attempt_ceiling_applies_after_previous_attempt_completes() {
    let mut p = policy();
    p.max_extra_attempts_per_operation = 1;
    let mut budget = RetryBudget::new(p).unwrap();
    begin(&mut budget, 1, 7);
    let first = retry(&mut budget, 1, 7);
    budget.complete_extra(&first).unwrap();
    assert_eq!(
        budget.reserve_extra(
            1, 7, RetryMode::SequentialRetry, RetryEligibility::SequentialAndHedge
        ),
        Err(BudgetError::OperationExtraLimit)
    );
}

#[test]
fn finishing_operation_with_inflight_extra_is_rejected() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    begin(&mut budget, 1, 7);
    let permit = retry(&mut budget, 1, 7);
    assert_eq!(
        budget.finish_operation(1),
        Err(BudgetError::OperationHasInflightExtra)
    );
    budget.complete_extra(&permit).unwrap();
    budget.finish_operation(1).unwrap();
    assert_eq!(budget.active_operation_count(), 0);
}

#[test]
fn finishing_operation_releases_global_and_workload_capacity() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    begin(&mut budget, 1, 7);
    let permit = retry(&mut budget, 1, 7);
    budget.complete_extra(&permit).unwrap();
    budget.finish_operation(1).unwrap();
    begin(&mut budget, 2, 8);
    let next = retry(&mut budget, 2, 8);
    assert_eq!(budget.active_extra_count(), 1);
    budget.complete_extra(&next).unwrap();
}

#[test]
fn operation_cannot_reserve_for_another_workload() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    begin(&mut budget, 1, 7);
    assert_eq!(
        budget.reserve_extra(
            1, 8, RetryMode::SequentialRetry, RetryEligibility::SequentialAndHedge
        ),
        Err(BudgetError::WorkloadMismatch)
    );
}

#[test]
fn unknown_operation_cannot_reserve_budget() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    assert_eq!(
        budget.reserve_extra(
            1, 7, RetryMode::SequentialRetry, RetryEligibility::SequentialAndHedge
        ),
        Err(BudgetError::OperationNotActive)
    );
}

#[test]
fn duplicate_logical_operation_cannot_be_opened_twice() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    begin(&mut budget, 1, 7);
    assert_eq!(
        budget.begin_operation(1, 7),
        Err(BudgetError::OperationAlreadyActive)
    );
}

#[test]
fn extra_permit_is_single_use() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    begin(&mut budget, 1, 7);
    let permit = retry(&mut budget, 1, 7);
    budget.complete_extra(&permit).unwrap();
    assert_eq!(
        budget.complete_extra(&permit),
        Err(BudgetError::UnknownPermit)
    );
}

#[test]
fn hedge_concurrency_has_an_independent_ceiling() {
    let mut p = policy();
    p.max_hedges_in_flight = 1;
    p.max_global_extra_in_flight = 10;
    p.global_ratio_numerator = 10;
    p.workload_ratio_numerator = 10;
    let mut budget = RetryBudget::new(p).unwrap();
    begin(&mut budget, 1, 7);
    begin(&mut budget, 2, 7);
    let first = budget.reserve_extra(
        1, 7, RetryMode::Hedge, RetryEligibility::SequentialAndHedge
    ).unwrap();
    assert_eq!(
        budget.reserve_extra(
            2, 7, RetryMode::Hedge, RetryEligibility::SequentialAndHedge
        ),
        Err(BudgetError::HedgeInflightLimit)
    );
    budget.complete_extra(&first).unwrap();
}

#[test]
fn retries_and_hedges_share_the_global_extra_attempt_limit() {
    let mut p = policy();
    p.max_global_extra_in_flight = 1;
    p.global_ratio_numerator = 10;
    p.workload_ratio_numerator = 10;
    let mut budget = RetryBudget::new(p).unwrap();
    begin(&mut budget, 1, 7);
    begin(&mut budget, 2, 7);
    let retry_permit = retry(&mut budget, 1, 7);
    assert_eq!(
        budget.reserve_extra(
            2, 7, RetryMode::Hedge, RetryEligibility::SequentialAndHedge
        ),
        Err(BudgetError::GlobalInflightLimit)
    );
    budget.complete_extra(&retry_permit).unwrap();
}

#[test]
fn zero_ratio_denominator_is_invalid_policy() {
    let mut p = policy();
    p.global_ratio_denominator = 0;
    assert_eq!(
        RetryBudget::new(p).err(),
        Some(BudgetError::InvalidPolicy)
    );
}

#[test]
fn generated_workload_interleavings_respect_global_and_tenant_caps() {
    let mut p = policy();
    p.global_ratio_numerator = 10;
    p.workload_ratio_numerator = 10;
    p.max_global_extra_in_flight = 12;
    p.max_workload_extra_in_flight = 2;
    p.max_extra_attempts_per_operation = 1;
    let mut budget = RetryBudget::new(p).unwrap();

    for operation_id in 1..=24 {
        let workload_id = if operation_id % 2 == 0 { 7 } else { 8 };
        begin(&mut budget, operation_id, workload_id);
    }

    let mut permits = Vec::new();
    for operation_id in 1..=24 {
        let workload_id = if operation_id % 2 == 0 { 7 } else { 8 };
        if let Ok(permit) = budget.reserve_extra(
            operation_id,
            workload_id,
            RetryMode::SequentialRetry,
            RetryEligibility::SequentialAndHedge,
        ) {
            permits.push(permit);
        }
    }
    assert_eq!(permits.len(), 4);
    assert!(permits.iter().filter(|p| p.workload_id == 7).count() <= 2);
    assert!(permits.iter().filter(|p| p.workload_id == 8).count() <= 2);
    for permit in &permits {
        budget.complete_extra(permit).unwrap();
    }
    assert_eq!(budget.active_extra_count(), 0);
}

#[test]
fn maximum_ratio_arithmetic_does_not_overflow() {
    assert_eq!(
        ratio_cap(u64::MAX, u64::MAX, 1, 0),
        u64::MAX
    );
}

#[test]
fn zero_identity_cannot_open_retry_budget_operation() {
    let mut budget = RetryBudget::new(policy()).unwrap();
    assert_eq!(
        budget.begin_operation(0, 7),
        Err(BudgetError::InvalidIdentity)
    );
    assert_eq!(
        budget.begin_operation(1, 0),
        Err(BudgetError::InvalidIdentity)
    );
}
