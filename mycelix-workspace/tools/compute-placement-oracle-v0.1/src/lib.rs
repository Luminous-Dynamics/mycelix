//! Exact small-instance oracle for federated compute placement.
//!
//! The solver enumerates feasible batch placements under declared CPU, memory,
//! GPU, and data-region constraints. It returns a Pareto frontier rather than
//! collapsing cost, data movement, failure-domain diversity, and resource
//! pressure into one score.
//!
//! This crate is intentionally local and pure: it does not discover providers,
//! authenticate inventory, issue leases, dispatch work, or verify outputs.
//! Its inputs are caller-supplied assertions, not independently verified facts.
//! A truncated search never returns an "optimal" status.

use std::collections::BTreeSet;

const CPU_MILLIS_PER_VCPU_HOUR: u128 = 3_600_000_000;
const GPU_MILLIS_PER_SLOT_HOUR: u128 = 3_600_000;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Worker {
    pub worker_id: String,
    pub operator_id: String,
    pub region_id: String,
    pub failure_domain_id: String,
    pub cpu_millis: u64,
    pub memory_mib: u64,
    pub gpu_slots: u32,
    /// Micro-USD per vCPU-hour, supplied by the benchmark instance.
    pub cpu_price_microusd_per_vcpu_hour: u64,
    /// Micro-USD per GPU-slot-hour, supplied by the benchmark instance.
    pub gpu_price_microusd_per_slot_hour: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Task {
    pub task_id: String,
    /// Hard data-rights/locality constraint. Empty means invalid, not unrestricted.
    pub permitted_regions: Vec<String>,
    pub input_region: String,
    pub input_bytes: u64,
    pub cpu_millis: u64,
    pub memory_mib: u64,
    pub gpu_slots: u32,
    pub duration_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlacementProblem {
    pub workers: Vec<Worker>,
    pub tasks: Vec<Task>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SolveLimits {
    /// Maximum attempted (task, worker) branches.
    pub max_search_nodes: u64,
    /// Maximum retained non-dominated candidates.
    pub max_frontier_size: usize,
}

impl Default for SolveLimits {
    fn default() -> Self {
        Self {
            max_search_nodes: 250_000,
            max_frontier_size: 128,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SolveError {
    NoWorkers,
    NoTasks,
    InvalidSearchLimits,
    EmptyWorkerId,
    DuplicateWorkerId(String),
    EmptyOperatorId(String),
    EmptyWorkerRegion(String),
    EmptyFailureDomain(String),
    ZeroWorkerCpu(String),
    ZeroWorkerMemory(String),
    EmptyTaskId,
    DuplicateTaskId(String),
    EmptyTaskInputRegion(String),
    NoPermittedRegions(String),
    EmptyPermittedRegion(String, String),
    DuplicatePermittedRegion(String, String),
    ZeroTaskCpu(String),
    ZeroTaskMemory(String),
    ZeroTaskDuration(String),
    ArithmeticOverflow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolveStatus {
    /// The complete finite search was explored and the frontier is exact.
    Optimal,
    /// The complete finite search was explored and no feasible placement exists.
    Infeasible,
    /// The node or frontier limit was reached; no optimality claim is permitted.
    Incomplete,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Assignment {
    pub task_id: String,
    pub worker_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlacementCandidate {
    pub assignments: Vec<Assignment>,
    pub estimated_cost_microusd: u128,
    pub remote_input_bytes: u128,
    /// Count of distinct failure domains used, not a probability of availability.
    pub distinct_failure_domains: usize,
    /// Count of declared operator identities; not proof of independent control.
    pub distinct_operators: usize,
    /// Descriptive count only; region spread is not a universal optimization preference.
    pub distinct_regions: usize,
    /// Peak CPU utilization across workers, in parts per million.
    pub peak_cpu_pressure_ppm: u64,
    /// Peak memory utilization across workers, in parts per million.
    pub peak_memory_pressure_ppm: u64,
    /// Peak GPU-slot utilization across workers, in parts per million.
    pub peak_gpu_pressure_ppm: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SolveReport {
    pub status: SolveStatus,
    pub explored_nodes: u64,
    pub feasible_assignments: u64,
    pub frontier: Vec<PlacementCandidate>,
}

#[derive(Debug, Clone, Copy, Default)]
struct ResourceUse {
    cpu_millis: u64,
    memory_mib: u64,
    gpu_slots: u32,
}

impl ResourceUse {
    fn fits(self, task: &Task, worker: &Worker) -> bool {
        self.cpu_millis
            .checked_add(task.cpu_millis)
            .is_some_and(|value| value <= worker.cpu_millis)
            && self
                .memory_mib
                .checked_add(task.memory_mib)
                .is_some_and(|value| value <= worker.memory_mib)
            && self
                .gpu_slots
                .checked_add(task.gpu_slots)
                .is_some_and(|value| value <= worker.gpu_slots)
    }

    fn add(&mut self, task: &Task) {
        self.cpu_millis += task.cpu_millis;
        self.memory_mib += task.memory_mib;
        self.gpu_slots += task.gpu_slots;
    }

    fn subtract(&mut self, task: &Task) {
        self.cpu_millis -= task.cpu_millis;
        self.memory_mib -= task.memory_mib;
        self.gpu_slots -= task.gpu_slots;
    }
}

fn validate_problem(problem: &PlacementProblem, limits: SolveLimits) -> Result<(), SolveError> {
    if problem.workers.is_empty() {
        return Err(SolveError::NoWorkers);
    }
    if problem.tasks.is_empty() {
        return Err(SolveError::NoTasks);
    }
    if limits.max_search_nodes == 0 || limits.max_frontier_size == 0 {
        return Err(SolveError::InvalidSearchLimits);
    }

    let mut worker_ids = BTreeSet::new();
    for worker in &problem.workers {
        if worker.worker_id.trim().is_empty() {
            return Err(SolveError::EmptyWorkerId);
        }
        if !worker_ids.insert(worker.worker_id.as_str()) {
            return Err(SolveError::DuplicateWorkerId(worker.worker_id.clone()));
        }
        if worker.operator_id.trim().is_empty() {
            return Err(SolveError::EmptyOperatorId(worker.worker_id.clone()));
        }
        if worker.region_id.trim().is_empty() {
            return Err(SolveError::EmptyWorkerRegion(worker.worker_id.clone()));
        }
        if worker.failure_domain_id.trim().is_empty() {
            return Err(SolveError::EmptyFailureDomain(worker.worker_id.clone()));
        }
        if worker.cpu_millis == 0 {
            return Err(SolveError::ZeroWorkerCpu(worker.worker_id.clone()));
        }
        if worker.memory_mib == 0 {
            return Err(SolveError::ZeroWorkerMemory(worker.worker_id.clone()));
        }
    }

    let mut task_ids = BTreeSet::new();
    for task in &problem.tasks {
        if task.task_id.trim().is_empty() {
            return Err(SolveError::EmptyTaskId);
        }
        if !task_ids.insert(task.task_id.as_str()) {
            return Err(SolveError::DuplicateTaskId(task.task_id.clone()));
        }
        if task.input_region.trim().is_empty() {
            return Err(SolveError::EmptyTaskInputRegion(task.task_id.clone()));
        }
        if task.permitted_regions.is_empty() {
            return Err(SolveError::NoPermittedRegions(task.task_id.clone()));
        }
        let mut regions = BTreeSet::new();
        for region in &task.permitted_regions {
            if region.trim().is_empty() {
                return Err(SolveError::EmptyPermittedRegion(
                    task.task_id.clone(),
                    region.clone(),
                ));
            }
            if !regions.insert(region.as_str()) {
                return Err(SolveError::DuplicatePermittedRegion(
                    task.task_id.clone(),
                    region.clone(),
                ));
            }
        }
        if task.cpu_millis == 0 {
            return Err(SolveError::ZeroTaskCpu(task.task_id.clone()));
        }
        if task.memory_mib == 0 {
            return Err(SolveError::ZeroTaskMemory(task.task_id.clone()));
        }
        if task.duration_ms == 0 {
            return Err(SolveError::ZeroTaskDuration(task.task_id.clone()));
        }
    }

    Ok(())
}

fn region_permitted(task: &Task, worker: &Worker) -> bool {
    task.permitted_regions
        .iter()
        .any(|region| region == &worker.region_id)
}

fn worker_can_fit_one(task: &Task, worker: &Worker) -> bool {
    region_permitted(task, worker)
        && task.cpu_millis <= worker.cpu_millis
        && task.memory_mib <= worker.memory_mib
        && task.gpu_slots <= worker.gpu_slots
}

fn ceil_div(numerator: u128, denominator: u128) -> u128 {
    numerator / denominator + if numerator % denominator == 0 { 0 } else { 1 }
}

fn checked_task_cost(task: &Task, worker: &Worker) -> Result<u128, SolveError> {
    let cpu_product = u128::from(worker.cpu_price_microusd_per_vcpu_hour)
        .checked_mul(u128::from(task.cpu_millis))
        .and_then(|value| value.checked_mul(u128::from(task.duration_ms)))
        .ok_or(SolveError::ArithmeticOverflow)?;
    let gpu_product = u128::from(worker.gpu_price_microusd_per_slot_hour)
        .checked_mul(u128::from(task.gpu_slots))
        .and_then(|value| value.checked_mul(u128::from(task.duration_ms)))
        .ok_or(SolveError::ArithmeticOverflow)?;

    Ok(ceil_div(cpu_product, CPU_MILLIS_PER_VCPU_HOUR)
        .checked_add(ceil_div(gpu_product, GPU_MILLIS_PER_SLOT_HOUR))
        .ok_or(SolveError::ArithmeticOverflow)?)
}

fn pressure_ppm(used: u64, capacity: u64) -> u64 {
    if used == 0 || capacity == 0 {
        return 0;
    }
    let numerator = u128::from(used) * 1_000_000;
    ceil_div(numerator, u128::from(capacity)).min(1_000_000) as u64
}

fn dominates(left: &PlacementCandidate, right: &PlacementCandidate) -> bool {
    let no_worse = left.estimated_cost_microusd <= right.estimated_cost_microusd
        && left.remote_input_bytes <= right.remote_input_bytes
        && left.distinct_failure_domains >= right.distinct_failure_domains
        && left.distinct_operators >= right.distinct_operators
        && left.peak_cpu_pressure_ppm <= right.peak_cpu_pressure_ppm
        && left.peak_memory_pressure_ppm <= right.peak_memory_pressure_ppm
        && left.peak_gpu_pressure_ppm <= right.peak_gpu_pressure_ppm;
    let strictly_better = left.estimated_cost_microusd < right.estimated_cost_microusd
        || left.remote_input_bytes < right.remote_input_bytes
        || left.distinct_failure_domains > right.distinct_failure_domains
        || left.distinct_operators > right.distinct_operators
        || left.peak_cpu_pressure_ppm < right.peak_cpu_pressure_ppm
        || left.peak_memory_pressure_ppm < right.peak_memory_pressure_ppm
        || left.peak_gpu_pressure_ppm < right.peak_gpu_pressure_ppm;
    no_worse && strictly_better
}

fn same_metrics(left: &PlacementCandidate, right: &PlacementCandidate) -> bool {
    left.estimated_cost_microusd == right.estimated_cost_microusd
        && left.remote_input_bytes == right.remote_input_bytes
        && left.distinct_failure_domains == right.distinct_failure_domains
        && left.distinct_operators == right.distinct_operators
        && left.distinct_regions == right.distinct_regions
        && left.peak_cpu_pressure_ppm == right.peak_cpu_pressure_ppm
        && left.peak_memory_pressure_ppm == right.peak_memory_pressure_ppm
        && left.peak_gpu_pressure_ppm == right.peak_gpu_pressure_ppm
}

fn sort_frontier(frontier: &mut [PlacementCandidate]) {
    frontier.sort_by(|left, right| {
        left.estimated_cost_microusd
            .cmp(&right.estimated_cost_microusd)
            .then_with(|| left.remote_input_bytes.cmp(&right.remote_input_bytes))
            .then_with(|| {
                right
                    .distinct_failure_domains
                    .cmp(&left.distinct_failure_domains)
            })
            .then_with(|| right.distinct_operators.cmp(&left.distinct_operators))
            .then_with(|| left.peak_cpu_pressure_ppm.cmp(&right.peak_cpu_pressure_ppm))
            .then_with(|| left.peak_memory_pressure_ppm.cmp(&right.peak_memory_pressure_ppm))
            .then_with(|| left.peak_gpu_pressure_ppm.cmp(&right.peak_gpu_pressure_ppm))
            .then_with(|| left.assignments.cmp(&right.assignments))
    });
}

struct SearchState<'a> {
    workers: Vec<&'a Worker>,
    tasks: Vec<&'a Task>,
    limits: SolveLimits,
    usage: Vec<ResourceUse>,
    chosen: Vec<(usize, usize)>,
    explored_nodes: u64,
    feasible_assignments: u64,
    frontier: Vec<PlacementCandidate>,
    incomplete: bool,
}

impl SearchState<'_> {
    fn recurse(&mut self, depth: usize) -> Result<(), SolveError> {
        if self.incomplete {
            return Ok(());
        }
        if depth == self.tasks.len() {
            self.record_candidate()?;
            return Ok(());
        }

        let task = self.tasks[depth];
        for worker_index in 0..self.workers.len() {
            if self.explored_nodes >= self.limits.max_search_nodes {
                self.incomplete = true;
                return Ok(());
            }
            self.explored_nodes += 1;

            let worker = self.workers[worker_index];
            if !region_permitted(task, worker) || !self.usage[worker_index].fits(task, worker) {
                continue;
            }

            self.usage[worker_index].add(task);
            self.chosen.push((depth, worker_index));
            self.recurse(depth + 1)?;
            self.chosen.pop();
            self.usage[worker_index].subtract(task);

            if self.incomplete {
                return Ok(());
            }
        }
        Ok(())
    }

    fn record_candidate(&mut self) -> Result<(), SolveError> {
        self.feasible_assignments += 1;

        let mut estimated_cost_microusd = 0_u128;
        let mut remote_input_bytes = 0_u128;
        let mut failure_domains = BTreeSet::new();
        let mut operators = BTreeSet::new();
        let mut regions = BTreeSet::new();
        let mut assignments = Vec::with_capacity(self.chosen.len());
        for (task_index, worker_index) in &self.chosen {
            let task = self.tasks[*task_index];
            let worker = self.workers[*worker_index];
            estimated_cost_microusd = estimated_cost_microusd
                .checked_add(checked_task_cost(task, worker)?)
                .ok_or(SolveError::ArithmeticOverflow)?;
            if task.input_region != worker.region_id {
                remote_input_bytes = remote_input_bytes
                    .checked_add(u128::from(task.input_bytes))
                    .ok_or(SolveError::ArithmeticOverflow)?;
            }
            failure_domains.insert(worker.failure_domain_id.as_str());
            operators.insert(worker.operator_id.as_str());
            regions.insert(worker.region_id.as_str());
            assignments.push(Assignment {
                task_id: task.task_id.clone(),
                worker_id: worker.worker_id.clone(),
            });
        }
        assignments.sort();

        let mut peak_cpu_pressure_ppm = 0_u64;
        let mut peak_memory_pressure_ppm = 0_u64;
        let mut peak_gpu_pressure_ppm = 0_u64;
        for (index, usage) in self.usage.iter().enumerate() {
            let worker = self.workers[index];
            peak_cpu_pressure_ppm =
                peak_cpu_pressure_ppm.max(pressure_ppm(usage.cpu_millis, worker.cpu_millis));
            peak_memory_pressure_ppm =
                peak_memory_pressure_ppm.max(pressure_ppm(usage.memory_mib, worker.memory_mib));
            peak_gpu_pressure_ppm = peak_gpu_pressure_ppm
                .max(pressure_ppm(u64::from(usage.gpu_slots), u64::from(worker.gpu_slots)));
        }

        let candidate = PlacementCandidate {
            assignments,
            estimated_cost_microusd,
            remote_input_bytes,
            distinct_failure_domains: failure_domains.len(),
            distinct_operators: operators.len(),
            distinct_regions: regions.len(),
            peak_cpu_pressure_ppm,
            peak_memory_pressure_ppm,
            peak_gpu_pressure_ppm,
        };

        if self.frontier.iter().any(|existing| {
            dominates(existing, &candidate)
                || (same_metrics(existing, &candidate)
                    && existing.assignments <= candidate.assignments)
        }) {
            return Ok(());
        }

        self.frontier.retain(|existing| {
            !dominates(&candidate, existing)
                && !(same_metrics(&candidate, existing)
                    && candidate.assignments < existing.assignments)
        });
        self.frontier.push(candidate);
        sort_frontier(&mut self.frontier);
        if self.frontier.len() > self.limits.max_frontier_size {
            self.frontier.truncate(self.limits.max_frontier_size);
            self.incomplete = true;
        }
        Ok(())
    }
}

/// Enumerate exact batch placements for small, caller-supplied instances.
///
/// The solver treats region permissions as hard constraints. It reports cost,
/// remote input bytes, failure-domain spread and peak resource pressure as
/// separate dimensions. It never authorizes or dispatches the returned plan.
pub fn solve_exact(
    problem: &PlacementProblem,
    limits: SolveLimits,
) -> Result<SolveReport, SolveError> {
    validate_problem(problem, limits)?;

    let mut workers: Vec<&Worker> = problem.workers.iter().collect();
    workers.sort_by(|left, right| left.worker_id.cmp(&right.worker_id));

    let mut tasks: Vec<&Task> = problem.tasks.iter().collect();
    tasks.sort_by(|left, right| {
        let left_eligible = workers
            .iter()
            .filter(|worker| worker_can_fit_one(left, worker))
            .count();
        let right_eligible = workers
            .iter()
            .filter(|worker| worker_can_fit_one(right, worker))
            .count();
        left_eligible
            .cmp(&right_eligible)
            .then_with(|| right.gpu_slots.cmp(&left.gpu_slots))
            .then_with(|| right.cpu_millis.cmp(&left.cpu_millis))
            .then_with(|| right.memory_mib.cmp(&left.memory_mib))
            .then_with(|| left.task_id.cmp(&right.task_id))
    });

    if tasks
        .iter()
        .any(|task| !workers.iter().any(|worker| worker_can_fit_one(task, worker)))
    {
        return Ok(SolveReport {
            status: SolveStatus::Infeasible,
            explored_nodes: 0,
            feasible_assignments: 0,
            frontier: Vec::new(),
        });
    }

    let mut state = SearchState {
        usage: vec![ResourceUse::default(); workers.len()],
        workers,
        tasks,
        limits,
        chosen: Vec::new(),
        explored_nodes: 0,
        feasible_assignments: 0,
        frontier: Vec::new(),
        incomplete: false,
    };
    state.recurse(0)?;

    let status = if state.incomplete {
        SolveStatus::Incomplete
    } else if state.feasible_assignments == 0 {
        SolveStatus::Infeasible
    } else {
        SolveStatus::Optimal
    };

    Ok(SolveReport {
        status,
        explored_nodes: state.explored_nodes,
        feasible_assignments: state.feasible_assignments,
        frontier: state.frontier,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn example_problem() -> PlacementProblem {
        PlacementProblem {
            workers: vec![
                Worker {
                    worker_id: "worker-west".into(),
                    operator_id: "operator-west".into(),
                    region_id: "region-west".into(),
                    failure_domain_id: "fd-west-1".into(),
                    cpu_millis: 8_000,
                    memory_mib: 16_384,
                    gpu_slots: 1,
                    cpu_price_microusd_per_vcpu_hour: 1_000,
                    gpu_price_microusd_per_slot_hour: 2_000,
                },
                Worker {
                    worker_id: "worker-east".into(),
                    operator_id: "operator-east".into(),
                    region_id: "region-east".into(),
                    failure_domain_id: "fd-east-1".into(),
                    cpu_millis: 16_000,
                    memory_mib: 32_768,
                    gpu_slots: 2,
                    cpu_price_microusd_per_vcpu_hour: 500,
                    gpu_price_microusd_per_slot_hour: 1_000,
                },
            ],
            tasks: vec![
                Task {
                    task_id: "task-a".into(),
                    permitted_regions: vec!["region-west".into(), "region-east".into()],
                    input_region: "region-west".into(),
                    input_bytes: 10 * 1_024 * 1_024 * 1_024,
                    cpu_millis: 4_000,
                    memory_mib: 8_192,
                    gpu_slots: 1,
                    duration_ms: 3_600_000,
                },
                Task {
                    task_id: "task-b".into(),
                    permitted_regions: vec!["region-east".into()],
                    input_region: "region-east".into(),
                    input_bytes: 512 * 1_024 * 1_024,
                    cpu_millis: 8_000,
                    memory_mib: 16_384,
                    gpu_slots: 1,
                    duration_ms: 1_800_000,
                },
            ],
        }
    }

    #[test]
    fn exact_frontier_preserves_cost_locality_and_failure_domain_tradeoff() {
        let report = solve_exact(&example_problem(), SolveLimits::default()).unwrap();
        assert_eq!(report.status, SolveStatus::Optimal);
        assert_eq!(report.feasible_assignments, 2);
        assert_eq!(report.frontier.len(), 2);

        let local_diverse = report
            .frontier
            .iter()
            .find(|candidate| candidate.remote_input_bytes == 0)
            .unwrap();
        assert_eq!(local_diverse.estimated_cost_microusd, 8_500);
        assert_eq!(local_diverse.distinct_failure_domains, 2);
        assert_eq!(local_diverse.distinct_operators, 2);
        assert_eq!(local_diverse.distinct_regions, 2);

        let low_cost_remote = report
            .frontier
            .iter()
            .find(|candidate| candidate.remote_input_bytes > 0)
            .unwrap();
        assert_eq!(low_cost_remote.estimated_cost_microusd, 5_500);
        assert_eq!(low_cost_remote.remote_input_bytes, 10 * 1_024_u128.pow(3));
        assert_eq!(low_cost_remote.distinct_failure_domains, 1);
        assert_eq!(low_cost_remote.distinct_operators, 1);
        assert_eq!(low_cost_remote.distinct_regions, 1);
    }


    #[test]
    fn distinct_failure_domains_do_not_imply_distinct_operators() {
        let mut problem = example_problem();
        problem.workers[1].operator_id = problem.workers[0].operator_id.clone();
        let report = solve_exact(&problem, SolveLimits::default()).unwrap();
        let diverse = report.frontier.iter()
            .find(|candidate| candidate.remote_input_bytes == 0)
            .unwrap();
        assert_eq!(diverse.distinct_failure_domains, 2);
        assert_eq!(diverse.distinct_operators, 1);
        assert_eq!(diverse.distinct_regions, 2);
    }

    #[test]
    fn region_diversity_is_reported_but_not_optimized_as_a_preference() {
        let problem = PlacementProblem {
            workers: vec![
                Worker {
                    worker_id: "worker-west-a".into(),
                    operator_id: "operator-shared".into(),
                    region_id: "region-west".into(),
                    failure_domain_id: "fd-shared".into(),
                    cpu_millis: 2_000,
                    memory_mib: 4_096,
                    gpu_slots: 0,
                    cpu_price_microusd_per_vcpu_hour: 1_000,
                    gpu_price_microusd_per_slot_hour: 0,
                },
                Worker {
                    worker_id: "worker-west-b".into(),
                    operator_id: "operator-shared".into(),
                    region_id: "region-west".into(),
                    failure_domain_id: "fd-shared".into(),
                    cpu_millis: 2_000,
                    memory_mib: 4_096,
                    gpu_slots: 0,
                    cpu_price_microusd_per_vcpu_hour: 1_000,
                    gpu_price_microusd_per_slot_hour: 0,
                },
                Worker {
                    worker_id: "worker-east".into(),
                    operator_id: "operator-shared".into(),
                    region_id: "region-east".into(),
                    failure_domain_id: "fd-shared".into(),
                    cpu_millis: 2_000,
                    memory_mib: 4_096,
                    gpu_slots: 0,
                    cpu_price_microusd_per_vcpu_hour: 1_000,
                    gpu_price_microusd_per_slot_hour: 0,
                },
            ],
            tasks: vec![
                Task {
                    task_id: "task-fixed-west".into(),
                    permitted_regions: vec!["region-west".into()],
                    input_region: "region-west".into(),
                    input_bytes: 0,
                    cpu_millis: 500,
                    memory_mib: 1_024,
                    gpu_slots: 0,
                    duration_ms: 3_600_000,
                },
                Task {
                    task_id: "task-flexible".into(),
                    permitted_regions: vec!["region-west".into(), "region-east".into()],
                    input_region: "region-west".into(),
                    input_bytes: 0,
                    cpu_millis: 500,
                    memory_mib: 1_024,
                    gpu_slots: 0,
                    duration_ms: 3_600_000,
                },
            ],
        };

        let report = solve_exact(&problem, SolveLimits::default()).unwrap();
        assert_eq!(report.status, SolveStatus::Optimal);
        let candidates: Vec<_> = report.frontier.iter()
            .filter(|candidate| {
                candidate.estimated_cost_microusd == 1_000
                    && candidate.remote_input_bytes == 0
                    && candidate.distinct_failure_domains == 1
                    && candidate.distinct_operators == 1
                    && candidate.peak_cpu_pressure_ppm == 250_000
                    && candidate.peak_memory_pressure_ppm == 250_000
                    && candidate.peak_gpu_pressure_ppm == 0
            })
            .collect();
        assert_eq!(candidates.len(), 2);
        assert_eq!(
            candidates.iter().map(|candidate| candidate.distinct_regions).collect::<BTreeSet<_>>(),
            BTreeSet::from([1, 2])
        );
    }

    #[test]
    fn cpu_and_memory_pressure_tradeoffs_are_not_collapsed_into_one_maximum() {
        let worker_cpu = Worker {
            worker_id: "cpu-heavy".into(),
            operator_id: "same-operator".into(),
            region_id: "same-region".into(),
            failure_domain_id: "same-failure-domain".into(),
            cpu_millis: 1_000,
            memory_mib: 2_000,
            gpu_slots: 0,
            cpu_price_microusd_per_vcpu_hour: 1_000,
            gpu_price_microusd_per_slot_hour: 0,
        };
        let worker_memory = Worker {
            worker_id: "memory-heavy".into(),
            operator_id: "same-operator".into(),
            region_id: "same-region".into(),
            failure_domain_id: "same-failure-domain".into(),
            cpu_millis: 2_000,
            memory_mib: 1_000,
            gpu_slots: 0,
            cpu_price_microusd_per_vcpu_hour: 1_000,
            gpu_price_microusd_per_slot_hour: 0,
        };
        let task = Task {
            task_id: "balanced-task".into(),
            permitted_regions: vec!["same-region".into()],
            input_region: "same-region".into(),
            input_bytes: 0,
            cpu_millis: 1_000,
            memory_mib: 1_000,
            gpu_slots: 0,
            duration_ms: 3_600_000,
        };
        let report = solve_exact(
            &PlacementProblem {
                workers: vec![worker_cpu, worker_memory],
                tasks: vec![task],
            },
            SolveLimits::default(),
        ).unwrap();

        assert_eq!(report.status, SolveStatus::Optimal);
        assert_eq!(report.frontier.len(), 2);
        assert!(report.frontier.iter().any(|candidate| {
            candidate.peak_cpu_pressure_ppm == 1_000_000
                && candidate.peak_memory_pressure_ppm == 500_000
                && candidate.peak_gpu_pressure_ppm == 0
        }));
        assert!(report.frontier.iter().any(|candidate| {
            candidate.peak_cpu_pressure_ppm == 500_000
                && candidate.peak_memory_pressure_ppm == 1_000_000
                && candidate.peak_gpu_pressure_ppm == 0
        }));
    }

    #[test]
    fn region_permission_is_a_hard_constraint() {
        let mut problem = example_problem();
        problem.workers.retain(|worker| worker.worker_id == "worker-west");
        let report = solve_exact(&problem, SolveLimits::default()).unwrap();
        assert_eq!(report.status, SolveStatus::Infeasible);
        assert!(report.frontier.is_empty());
    }

    #[test]
    fn aggregate_capacity_prevents_double_booking() {
        let mut problem = example_problem();
        problem.workers.retain(|worker| worker.worker_id == "worker-west");
        problem.tasks[1].permitted_regions = vec!["region-west".into()];
        problem.tasks[1].gpu_slots = 1;
        assert_eq!(
            solve_exact(&problem, SolveLimits::default()).unwrap().status,
            SolveStatus::Infeasible
        );
    }

    #[test]
    fn exhausted_search_budget_never_claims_optimality() {
        let report = solve_exact(
            &example_problem(),
            SolveLimits {
                max_search_nodes: 1,
                max_frontier_size: 16,
            },
        )
        .unwrap();
        assert_eq!(report.status, SolveStatus::Incomplete);
    }

    #[test]
    fn reordered_input_produces_identical_exact_frontier() {
        let problem = example_problem();
        let expected = solve_exact(&problem, SolveLimits::default()).unwrap();
        let mut reordered = example_problem();
        reordered.workers.reverse();
        reordered.tasks.reverse();
        let actual = solve_exact(&reordered, SolveLimits::default()).unwrap();
        assert_eq!(actual, expected);
    }

    #[test]
    fn duplicate_worker_identity_is_rejected() {
        let mut problem = example_problem();
        problem.workers[1].worker_id = problem.workers[0].worker_id.clone();
        assert_eq!(
            solve_exact(&problem, SolveLimits::default()),
            Err(SolveError::DuplicateWorkerId("worker-west".into()))
        );
    }

    #[test]
    fn zero_search_or_frontier_budget_is_rejected() {
        let problem = example_problem();
        assert_eq!(
            solve_exact(
                &problem,
                SolveLimits {
                    max_search_nodes: 0,
                    max_frontier_size: 8,
                }
            ),
            Err(SolveError::InvalidSearchLimits)
        );
        assert_eq!(
            solve_exact(
                &problem,
                SolveLimits {
                    max_search_nodes: 8,
                    max_frontier_size: 0,
                }
            ),
            Err(SolveError::InvalidSearchLimits)
        );
    }
}
