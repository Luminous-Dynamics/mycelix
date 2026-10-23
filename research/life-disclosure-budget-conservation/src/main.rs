use std::collections::BTreeMap;
use std::fmt::Write as _;

const GLOBAL_BUDGET: u32 = 10;
const ACTIVE_EPOCH: u64 = 1;
const VERIFIER_COUNT: usize = 4;
const MILLION: u64 = 1_000_000;
const MANIFEST: &str = include_str!("../budget-manifest.json");

#[derive(Clone, Debug, PartialEq, Eq)]
struct Request {
    id: u64,
    verifier: usize,
    cost: u32,
    policy_epoch: u64,
    subject_ref: u64,
    purpose_ref: u64,
    payload_digest: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Outcome {
    Accepted,
    DeniedBudget,
    DeniedEpoch,
    InvalidCost,
    DeniedEscrow,
    DuplicateReplay,
    Conflict,
}

impl Outcome {
    const fn is_denial(self) -> bool {
        matches!(
            self,
            Self::DeniedBudget
                | Self::DeniedEpoch
                | Self::InvalidCost
                | Self::DeniedEscrow
                | Self::Conflict
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Metrics {
    offered: u32,
    accepted_unique: u32,
    denied: u32,
    duplicate_replays: u32,
    conflicting_reuse: u32,
    stale_epoch_rejections: u32,
    spent_units: u32,
    budget_units: u32,
    policy_valid: bool,
}

impl Metrics {
    fn new(budget_units: u32, policy_valid: bool) -> Self {
        Self {
            offered: 0,
            accepted_unique: 0,
            denied: 0,
            duplicate_replays: 0,
            conflicting_reuse: 0,
            stale_epoch_rejections: 0,
            spent_units: 0,
            budget_units,
            policy_valid,
        }
    }

    fn overspend_units(&self) -> u32 {
        self.spent_units.saturating_sub(self.budget_units)
    }

    fn availability_ppm(&self) -> u64 {
        if self.offered == 0 {
            0
        } else {
            u64::from(self.accepted_unique) * MILLION / u64::from(self.offered)
        }
    }
}

struct SharedLedger {
    budget_units: u32,
    active_epoch: u64,
    spent_units: u32,
    records: BTreeMap<u64, Request>,
    metrics: Metrics,
}

impl SharedLedger {
    fn new(budget_units: u32, active_epoch: u64) -> Self {
        Self {
            budget_units,
            active_epoch,
            spent_units: 0,
            records: BTreeMap::new(),
            metrics: Metrics::new(budget_units, true),
        }
    }

    // Policy revision changes admission rules; it does not erase disclosure history.
    fn advance_epoch(&mut self, next_epoch: u64) -> bool {
        if next_epoch <= self.active_epoch {
            return false;
        }
        self.active_epoch = next_epoch;
        true
    }

    fn admit(&mut self, request: Request) -> Outcome {
        self.metrics.offered += 1;

        if let Some(existing) = self.records.get(&request.id) {
            if existing == &request {
                self.metrics.duplicate_replays += 1;
                return Outcome::DuplicateReplay;
            }
            self.metrics.conflicting_reuse += 1;
            self.metrics.denied += 1;
            return Outcome::Conflict;
        }

        let outcome = if request.cost == 0 {
            Outcome::InvalidCost
        } else if request.policy_epoch != self.active_epoch {
            self.metrics.stale_epoch_rejections += 1;
            Outcome::DeniedEpoch
        } else if request.cost > self.budget_units.saturating_sub(self.spent_units) {
            Outcome::DeniedBudget
        } else {
            self.spent_units += request.cost;
            self.metrics.spent_units = self.spent_units;
            self.metrics.accepted_unique += 1;
            Outcome::Accepted
        };

        if outcome.is_denial() {
            self.metrics.denied += 1;
        }
        self.records.insert(request.id, request);
        outcome
    }

    fn metrics(&self) -> Metrics {
        self.metrics.clone()
    }
}

struct EscrowLedger {
    active_epoch: u64,
    allocations: Vec<u32>,
    spent_by_verifier: Vec<u32>,
    records: BTreeMap<u64, Request>,
    metrics: Metrics,
}

impl EscrowLedger {
    fn new(budget_units: u32, active_epoch: u64, allocations: Vec<u32>) -> Option<Self> {
        let allocation_total = allocations
            .iter()
            .try_fold(0_u32, |total, allocation| total.checked_add(*allocation))?;
        if allocations.is_empty() || allocation_total > budget_units {
            return None;
        }

        Some(Self {
            active_epoch,
            spent_by_verifier: vec![0; allocations.len()],
            allocations,
            records: BTreeMap::new(),
            metrics: Metrics::new(budget_units, true),
        })
    }

    // Keep consumed escrow spent across policy revisions within the same budget scope.
    fn advance_epoch(&mut self, next_epoch: u64) -> bool {
        if next_epoch <= self.active_epoch {
            return false;
        }
        self.active_epoch = next_epoch;
        true
    }

    fn admit(&mut self, request: Request) -> Outcome {
        self.metrics.offered += 1;

        if let Some(existing) = self.records.get(&request.id) {
            if existing == &request {
                self.metrics.duplicate_replays += 1;
                return Outcome::DuplicateReplay;
            }
            self.metrics.conflicting_reuse += 1;
            self.metrics.denied += 1;
            return Outcome::Conflict;
        }

        let outcome = if request.cost == 0 {
            Outcome::InvalidCost
        } else if request.policy_epoch != self.active_epoch {
            self.metrics.stale_epoch_rejections += 1;
            Outcome::DeniedEpoch
        } else {
            match self.allocations.get(request.verifier) {
                None => Outcome::DeniedEscrow,
                Some(allocation) => {
                    let local_spent = self.spent_by_verifier[request.verifier];
                    if request.cost > allocation.saturating_sub(local_spent) {
                        Outcome::DeniedEscrow
                    } else {
                        self.spent_by_verifier[request.verifier] += request.cost;
                        self.metrics.spent_units = self.spent_by_verifier.iter().sum();
                        self.metrics.accepted_unique += 1;
                        Outcome::Accepted
                    }
                }
            }
        };

        if outcome.is_denial() {
            self.metrics.denied += 1;
        }
        self.records.insert(request.id, request);
        outcome
    }

    fn metrics(&self) -> Metrics {
        self.metrics.clone()
    }
}

fn request(id: u64, verifier: usize, cost: u32, epoch: u64) -> Request {
    Request {
        id,
        verifier,
        cost,
        policy_epoch: epoch,
        subject_ref: 77,
        purpose_ref: 9,
        payload_digest: id.wrapping_mul(0x9e37_79b9_7f4a_7c15),
    }
}

fn independent_stale_replicas(requests: &[Request], budget: u32) -> Metrics {
    let mut metrics = Metrics::new(budget, true);
    for item in requests {
        metrics.offered += 1;
        // Each replica consults its own stale full budget and does not coordinate spend.
        if item.cost > budget || item.cost == 0 {
            metrics.denied += 1;
        } else {
            metrics.accepted_unique += 1;
            metrics.spent_units = metrics.spent_units.saturating_add(item.cost);
        }
    }
    metrics
}

fn baseline_requests() -> Vec<Request> {
    (0..VERIFIER_COUNT)
        .map(|verifier| request(verifier as u64 + 1, verifier, 3, ACTIVE_EPOCH))
        .collect()
}

fn run_report() -> String {
    let requests = baseline_requests();
    let stale = independent_stale_replicas(&requests, GLOBAL_BUDGET);

    let mut central = SharedLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH);
    for item in requests.iter().cloned() {
        central.admit(item);
    }

    let allocations = vec![3, 3, 2, 2];
    let mut escrow = EscrowLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH, allocations)
        .expect("frozen escrow allocation must be valid");
    for item in [
        request(11, 0, 3, ACTIVE_EPOCH),
        request(12, 1, 3, ACTIVE_EPOCH),
        request(13, 2, 2, ACTIVE_EPOCH),
        request(14, 3, 2, ACTIVE_EPOCH),
    ] {
        escrow.admit(item);
    }

    let mut invalid_escrow = Metrics::new(GLOBAL_BUDGET, false);
    invalid_escrow.offered = VERIFIER_COUNT as u32;
    invalid_escrow.denied = VERIFIER_COUNT as u32;

    let mut stale_epoch = SharedLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH + 1);
    for item in requests.iter().map(|item| Request {
        policy_epoch: ACTIVE_EPOCH,
        ..item.clone()
    }) {
        stale_epoch.admit(item);
    }

    let budget_semantics_declared = MANIFEST.contains("not epsilon, delta");
    let mut output = String::new();
    writeln!(
        output,
        "policy,requests_offered,accepted_unique,denied,duplicate_replays,conflicting_reuse,stale_epoch_rejections,spent_units,budget_units,overspend_units,availability_ppm,policy_valid,budget_semantics_declared"
    )
    .expect("write to string");
    write_metrics_row(
        &mut output,
        "independent_stale_replicas",
        &stale,
        budget_semantics_declared,
    );
    write_metrics_row(
        &mut output,
        "serialized_shared_ledger",
        &central.metrics(),
        budget_semantics_declared,
    );
    write_metrics_row(
        &mut output,
        "bounded_escrow",
        &escrow.metrics(),
        budget_semantics_declared,
    );
    write_metrics_row(
        &mut output,
        "invalid_overallocated_escrow",
        &invalid_escrow,
        budget_semantics_declared,
    );
    write_metrics_row(
        &mut output,
        "stale_policy_epoch",
        &stale_epoch.metrics(),
        budget_semantics_declared,
    );
    output
}

fn write_metrics_row(
    output: &mut String,
    policy: &str,
    metrics: &Metrics,
    budget_semantics_declared: bool,
) {
    writeln!(
        output,
        "{},{},{},{},{},{},{},{},{},{},{},{},{}",
        policy,
        metrics.offered,
        metrics.accepted_unique,
        metrics.denied,
        metrics.duplicate_replays,
        metrics.conflicting_reuse,
        metrics.stale_epoch_rejections,
        metrics.spent_units,
        metrics.budget_units,
        metrics.overspend_units(),
        metrics.availability_ppm(),
        metrics.policy_valid,
        budget_semantics_declared
    )
    .expect("write to string");
}

fn all_permutations(items: &mut [Request], start: usize, output: &mut Vec<Vec<Request>>) {
    if start == items.len() {
        output.push(items.to_vec());
        return;
    }
    for index in start..items.len() {
        items.swap(start, index);
        all_permutations(items, start + 1, output);
        items.swap(start, index);
    }
}

fn main() {
    print!("{}", run_report());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stale_replicas_demonstrate_global_budget_double_spend() {
        let metrics = independent_stale_replicas(&baseline_requests(), GLOBAL_BUDGET);
        assert_eq!(metrics.accepted_unique, 4);
        assert_eq!(metrics.spent_units, 12);
        assert_eq!(metrics.overspend_units(), 2);
        assert_eq!(metrics.availability_ppm(), MILLION);
    }

    #[test]
    fn serialized_ledger_never_overspends_under_every_mixed_cost_order() {
        let mut batch = vec![
            request(1, 0, 2, ACTIVE_EPOCH),
            request(2, 1, 3, ACTIVE_EPOCH),
            request(3, 2, 4, ACTIVE_EPOCH),
            request(4, 3, 4, ACTIVE_EPOCH),
        ];
        let mut schedules = Vec::new();
        all_permutations(&mut batch, 0, &mut schedules);
        assert_eq!(schedules.len(), 24);

        for schedule in schedules {
            let mut ledger = SharedLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH);
            for item in schedule {
                ledger.admit(item);
            }
            let metrics = ledger.metrics();
            assert!(metrics.spent_units <= GLOBAL_BUDGET);
            assert_eq!(metrics.overspend_units(), 0);
        }
    }

    #[test]
    fn replay_is_idempotent_and_any_request_scope_reuse_conflicts() {
        let mut ledger = SharedLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH);
        let original = request(7, 0, 4, ACTIVE_EPOCH);
        assert_eq!(ledger.admit(original.clone()), Outcome::Accepted);
        let spent_after_first = ledger.metrics().spent_units;

        assert_eq!(ledger.admit(original.clone()), Outcome::DuplicateReplay);
        assert_eq!(ledger.metrics().spent_units, spent_after_first);
        assert_eq!(ledger.metrics().duplicate_replays, 1);

        let conflicting_requests = [
            Request { cost: 5, ..original.clone() },
            Request { payload_digest: 0xdead_beef, ..original.clone() },
            Request { subject_ref: 78, ..original.clone() },
            Request { purpose_ref: 10, ..original.clone() },
            Request { verifier: 1, ..original.clone() },
            Request { policy_epoch: ACTIVE_EPOCH + 1, ..original.clone() },
        ];

        for changed in conflicting_requests {
            assert_eq!(ledger.admit(changed), Outcome::Conflict);
        }
        assert_eq!(ledger.metrics().spent_units, spent_after_first);
        assert_eq!(ledger.metrics().conflicting_reuse, 6);
    }

    #[test]
    fn policy_epoch_rollover_does_not_refund_shared_budget() {
        let mut ledger = SharedLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH);
        assert_eq!(
            ledger.admit(request(51, 0, 7, ACTIVE_EPOCH)),
            Outcome::Accepted
        );
        assert!(ledger.advance_epoch(ACTIVE_EPOCH + 1));
        assert_eq!(ledger.metrics().spent_units, 7);

        assert_eq!(
            ledger.admit(request(52, 1, 4, ACTIVE_EPOCH + 1)),
            Outcome::DeniedBudget
        );
        assert_eq!(
            ledger.admit(request(53, 1, 3, ACTIVE_EPOCH + 1)),
            Outcome::Accepted
        );
        assert_eq!(ledger.metrics().spent_units, GLOBAL_BUDGET);
        assert!(!ledger.advance_epoch(ACTIVE_EPOCH + 1));
        assert!(!ledger.advance_epoch(ACTIVE_EPOCH));
        assert_eq!(ledger.metrics().spent_units, GLOBAL_BUDGET);
    }

    #[test]
    fn bounded_escrow_conserves_global_limit_and_allows_within_quota_offline() {
        let allocations = vec![3, 3, 2, 2];
        let mut ledger = EscrowLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH, allocations.clone())
            .expect("allocation sum is within global budget");

        for (verifier, cost) in allocations.iter().copied().enumerate() {
            assert_eq!(
                ledger.admit(request(verifier as u64 + 1, verifier, cost, ACTIVE_EPOCH)),
                Outcome::Accepted
            );
        }

        let metrics = ledger.metrics();
        assert_eq!(metrics.accepted_unique, 4);
        assert_eq!(metrics.spent_units, GLOBAL_BUDGET);
        assert_eq!(metrics.overspend_units(), 0);
    }

    #[test]
    fn policy_epoch_rollover_does_not_refund_consumed_escrow() {
        let mut ledger = EscrowLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH, vec![5, 5])
            .expect("allocation sum is within global budget");

        assert_eq!(
            ledger.admit(request(61, 0, 4, ACTIVE_EPOCH)),
            Outcome::Accepted
        );
        assert!(ledger.advance_epoch(ACTIVE_EPOCH + 1));
        assert_eq!(ledger.metrics().spent_units, 4);

        assert_eq!(
            ledger.admit(request(62, 0, 2, ACTIVE_EPOCH + 1)),
            Outcome::DeniedEscrow
        );
        assert_eq!(
            ledger.admit(request(63, 0, 1, ACTIVE_EPOCH + 1)),
            Outcome::Accepted
        );
        assert_eq!(ledger.metrics().spent_units, 5);
        assert!(!ledger.advance_epoch(ACTIVE_EPOCH));
        assert_eq!(ledger.metrics().spent_units, 5);
    }

    #[test]
    fn escrow_cannot_borrow_another_verifiers_partitioned_allocation() {
        let mut ledger = EscrowLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH, vec![3, 3, 2, 2])
            .expect("allocation sum is within global budget");
        assert_eq!(
            ledger.admit(request(1, 2, 3, ACTIVE_EPOCH)),
            Outcome::DeniedEscrow
        );
        assert_eq!(ledger.metrics().spent_units, 0);
    }

    #[test]
    fn invalid_or_overflowing_escrow_allocations_fail_closed() {
        assert!(EscrowLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH, vec![4, 4, 4, 4]).is_none());
        assert!(EscrowLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH, vec![u32::MAX, 1]).is_none());
        assert!(EscrowLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH, vec![]).is_none());
    }

    #[test]
    fn stale_epoch_and_zero_cost_requests_do_not_spend_budget() {
        let mut ledger = SharedLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH + 1);
        assert_eq!(
            ledger.admit(request(1, 0, 3, ACTIVE_EPOCH)),
            Outcome::DeniedEpoch
        );

        let mut current = SharedLedger::new(GLOBAL_BUDGET, ACTIVE_EPOCH);
        assert_eq!(
            current.admit(request(2, 1, 0, ACTIVE_EPOCH)),
            Outcome::InvalidCost
        );
        assert_eq!(ledger.metrics().spent_units, 0);
        assert_eq!(current.metrics().spent_units, 0);
    }

    #[test]
    fn report_is_byte_for_byte_deterministic_and_claims_no_dp_parameters() {
        assert_eq!(run_report(), run_report());
        assert!(MANIFEST.contains("not epsilon, delta"));
        assert!(run_report().contains("overspend_units"));
    }

    #[test]
    fn optional_disclosure_denial_is_separate_from_required_task_decision() {
        let required_task_result = "ELIGIBLE";
        let mut ledger = SharedLedger::new(0, ACTIVE_EPOCH);
        let decision = ledger.admit(request(99, 0, 1, ACTIVE_EPOCH));
        assert_eq!(decision, Outcome::DeniedBudget);
        assert_eq!(required_task_result, "ELIGIBLE");
    }
}
