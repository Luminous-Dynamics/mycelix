//! Exact bounded placement for fixed task intervals.
//!
//! Every task occupies a worker during the half-open interval [start_ms, end_ms).
//! This kernel selects worker assignments; it does not choose task start times,
//! queue work, preempt jobs, dispatch execution, or verify outputs.

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
    pub cpu_price_microusd_per_vcpu_hour: u64,
    pub gpu_price_microusd_per_slot_hour: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Task {
    pub task_id: String,
    pub permitted_regions: Vec<String>,
    pub input_region: String,
    pub input_bytes: u64,
    pub cpu_millis: u64,
    pub memory_mib: u64,
    pub gpu_slots: u32,
    /// Inclusive start of the task's occupied time window.
    pub start_ms: u64,
    /// Exclusive end of the task's occupied time window.
    pub end_ms: u64,
}

impl Task {
    pub fn duration_ms(&self) -> u64 {
        self.end_ms - self.start_ms
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlacementProblem {
    pub workers: Vec<Worker>,
    pub tasks: Vec<Task>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SolveLimits {
    pub max_search_nodes: u64,
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
    InvalidTaskInterval(String),
    ArithmeticOverflow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolveStatus {
    Optimal,
    Infeasible,
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
    /// Number of distinct declared failure-domain IDs; not an availability probability.
    pub distinct_failure_domains: usize,
    /// Number of distinct declared operator IDs; not proof of independent control.
    pub distinct_operators: usize,
    /// Descriptive only; not included in Pareto dominance or preference ordering.
    pub distinct_regions: usize,
    /// Peak CPU utilization across all workers and time segments, in parts per million.
    pub peak_cpu_pressure_ppm: u64,
    /// Peak memory utilization across all workers and time segments, in parts per million.
    pub peak_memory_pressure_ppm: u64,
    /// Peak GPU-slot utilization across all workers and time segments, in parts per million.
    pub peak_gpu_pressure_ppm: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SolveReport {
    pub status: SolveStatus,
    pub explored_nodes: u64,
    pub feasible_assignments: u64,
    pub frontier: Vec<PlacementCandidate>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct ResourceUse {
    cpu_millis: u64,
    memory_mib: u64,
    gpu_slots: u32,
}

impl ResourceUse {
    fn checked_add(
        self,
        cpu_millis: u64,
        memory_mib: u64,
        gpu_slots: u32,
    ) -> Option<Self> {
        Some(Self {
            cpu_millis: self.cpu_millis.checked_add(cpu_millis)?,
            memory_mib: self.memory_mib.checked_add(memory_mib)?,
            gpu_slots: self.gpu_slots.checked_add(gpu_slots)?,
        })
    }

    fn checked_sub(
        self,
        cpu_millis: u64,
        memory_mib: u64,
        gpu_slots: u32,
    ) -> Option<Self> {
        Some(Self {
            cpu_millis: self.cpu_millis.checked_sub(cpu_millis)?,
            memory_mib: self.memory_mib.checked_sub(memory_mib)?,
            gpu_slots: self.gpu_slots.checked_sub(gpu_slots)?,
        })
    }

    fn fits(self, worker: &Worker) -> bool {
        self.cpu_millis <= worker.cpu_millis
            && self.memory_mib <= worker.memory_mib
            && self.gpu_slots <= worker.gpu_slots
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct ResourcePressure {
    cpu_ppm: u64,
    memory_ppm: u64,
    gpu_ppm: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum EventKind {
    /// End sorts before Start at an identical timestamp: [start, end) semantics.
    End,
    Start,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ResourceEvent {
    at_ms: u64,
    kind: EventKind,
    cpu_millis: u64,
    memory_mib: u64,
    gpu_slots: u32,
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
        if task.start_ms >= task.end_ms {
            return Err(SolveError::InvalidTaskInterval(task.task_id.clone()));
        }
    }
    Ok(())
}

fn region_permitted(task: &Task, worker: &Worker) -> bool {
    task.permitted_regions.iter().any(|region| region == &worker.region_id)
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
    let duration_ms = task.duration_ms();
    let cpu_product = u128::from(worker.cpu_price_microusd_per_vcpu_hour)
        .checked_mul(u128::from(task.cpu_millis))
        .and_then(|value| value.checked_mul(u128::from(duration_ms)))
        .ok_or(SolveError::ArithmeticOverflow)?;
    let gpu_product = u128::from(worker.gpu_price_microusd_per_slot_hour)
        .checked_mul(u128::from(task.gpu_slots))
        .and_then(|value| value.checked_mul(u128::from(duration_ms)))
        .ok_or(SolveError::ArithmeticOverflow)?;
    ceil_div(cpu_product, CPU_MILLIS_PER_VCPU_HOUR)
        .checked_add(ceil_div(gpu_product, GPU_MILLIS_PER_SLOT_HOUR))
        .ok_or(SolveError::ArithmeticOverflow)
}

fn pressure_ppm(used: u64, capacity: u64) -> u64 {
    if used == 0 || capacity == 0 {
        return 0;
    }
    let numerator = u128::from(used) * 1_000_000;
    ceil_div(numerator, u128::from(capacity)).min(1_000_000) as u64
}

fn worker_peak_pressure(tasks: &[&Task], worker: &Worker) -> Option<ResourcePressure> {
    let mut events = Vec::with_capacity(tasks.len().checked_mul(2)?);
    for task in tasks {
        events.push(ResourceEvent {
            at_ms: task.start_ms,
            kind: EventKind::Start,
            cpu_millis: task.cpu_millis,
            memory_mib: task.memory_mib,
            gpu_slots: task.gpu_slots,
        });
        events.push(ResourceEvent {
            at_ms: task.end_ms,
            kind: EventKind::End,
            cpu_millis: task.cpu_millis,
            memory_mib: task.memory_mib,
            gpu_slots: task.gpu_slots,
        });
    }
    events.sort_by_key(|event| (event.at_ms, event.kind));

    let mut used = ResourceUse::default();
    let mut peak = ResourcePressure::default();
    for event in events {
        used = match event.kind {
            EventKind::End => used.checked_sub(
                event.cpu_millis,
                event.memory_mib,
                event.gpu_slots,
            )?,
            EventKind::Start => used.checked_add(
                event.cpu_millis,
                event.memory_mib,
                event.gpu_slots,
            )?,
        };
        if !used.fits(worker) {
            return None;
        }
        peak.cpu_ppm = peak.cpu_ppm.max(pressure_ppm(used.cpu_millis, worker.cpu_millis));
        peak.memory_ppm = peak.memory_ppm.max(pressure_ppm(used.memory_mib, worker.memory_mib));
        peak.gpu_ppm = peak.gpu_ppm.max(pressure_ppm(
            u64::from(used.gpu_slots),
            u64::from(worker.gpu_slots),
        ));
    }
    Some(peak)
}

fn worker_can_host_additional(
    chosen: &[(usize, usize)],
    task_index: usize,
    worker_index: usize,
    tasks: &[&Task],
    workers: &[&Worker],
) -> bool {
    let mut worker_tasks: Vec<&Task> = chosen
        .iter()
        .filter(|(_, selected_worker)| *selected_worker == worker_index)
        .map(|(selected_task, _)| tasks[*selected_task])
        .collect();
    worker_tasks.push(tasks[task_index]);
    worker_peak_pressure(&worker_tasks, workers[worker_index]).is_some()
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
                right.distinct_failure_domains.cmp(&left.distinct_failure_domains)
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

        for worker_index in 0..self.workers.len() {
            if self.explored_nodes >= self.limits.max_search_nodes {
                self.incomplete = true;
                return Ok(());
            }
            self.explored_nodes += 1;

            if !region_permitted(self.tasks[depth], self.workers[worker_index]) {
                continue;
            }
            if !worker_can_host_additional(
                &self.chosen,
                depth,
                worker_index,
                &self.tasks,
                &self.workers,
            ) {
                continue;
            }

            self.chosen.push((depth, worker_index));
            self.recurse(depth + 1)?;
            self.chosen.pop();
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
        let mut peak_pressure = ResourcePressure::default();

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

        for (worker_index, worker) in self.workers.iter().enumerate() {
            let worker_tasks: Vec<&Task> = self.chosen
                .iter()
                .filter(|(_, selected_worker)| *selected_worker == worker_index)
                .map(|(task_index, _)| self.tasks[*task_index])
                .collect();
            let worker_peak = worker_peak_pressure(&worker_tasks, worker)
                .ok_or(SolveError::ArithmeticOverflow)?;
            peak_pressure.cpu_ppm = peak_pressure.cpu_ppm.max(worker_peak.cpu_ppm);
            peak_pressure.memory_ppm = peak_pressure.memory_ppm.max(worker_peak.memory_ppm);
            peak_pressure.gpu_ppm = peak_pressure.gpu_ppm.max(worker_peak.gpu_ppm);
        }

        let candidate = PlacementCandidate {
            assignments,
            estimated_cost_microusd,
            remote_input_bytes,
            distinct_failure_domains: failure_domains.len(),
            distinct_operators: operators.len(),
            distinct_regions: regions.len(),
            peak_cpu_pressure_ppm: peak_pressure.cpu_ppm,
            peak_memory_pressure_ppm: peak_pressure.memory_ppm,
            peak_gpu_pressure_ppm: peak_pressure.gpu_ppm,
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

/// Enumerate assignments for fixed task intervals using exact event-sweep checks.
///
/// Region permissions are hard constraints. End events are processed before
/// start events at the same time, so an interval ending at t releases capacity
/// for another interval beginning at t.
pub fn solve_exact(
    problem: &PlacementProblem,
    limits: SolveLimits,
) -> Result<SolveReport, SolveError> {
    validate_problem(problem, limits)?;

    let mut workers: Vec<&Worker> = problem.workers.iter().collect();
    workers.sort_by(|left, right| left.worker_id.cmp(&right.worker_id));
    let mut tasks: Vec<&Task> = problem.tasks.iter().collect();
    tasks.sort_by(|left, right| {
        let left_eligible = workers.iter().filter(|worker| worker_can_fit_one(left, worker)).count();
        let right_eligible = workers.iter().filter(|worker| worker_can_fit_one(right, worker)).count();
        left_eligible.cmp(&right_eligible)
            .then_with(|| right.duration_ms().cmp(&left.duration_ms()))
            .then_with(|| right.cpu_millis.cmp(&left.cpu_millis))
            .then_with(|| right.memory_mib.cmp(&left.memory_mib))
            .then_with(|| left.task_id.cmp(&right.task_id))
    });

    if tasks.iter().any(|task| {
        !workers.iter().any(|worker| worker_can_fit_one(task, worker))
    }) {
        return Ok(SolveReport {
            status: SolveStatus::Infeasible,
            explored_nodes: 0,
            feasible_assignments: 0,
            frontier: Vec::new(),
        });
    }

    let mut state = SearchState {
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

    fn worker(capacity: u64) -> Worker {
        Worker {
            worker_id: "worker-a".into(),
            operator_id: "operator-a".into(),
            region_id: "region-a".into(),
            failure_domain_id: "fd-a".into(),
            cpu_millis: capacity,
            memory_mib: 4_096,
            gpu_slots: 2,
            cpu_price_microusd_per_vcpu_hour: 1_000,
            gpu_price_microusd_per_slot_hour: 2_000,
        }
    }

    fn task(id: &str, start_ms: u64, end_ms: u64, cpu_millis: u64) -> Task {
        Task {
            task_id: id.into(),
            permitted_regions: vec!["region-a".into()],
            input_region: "region-a".into(),
            input_bytes: 0,
            cpu_millis,
            memory_mib: 512,
            gpu_slots: 0,
            start_ms,
            end_ms,
        }
    }

    fn problem(workers: Vec<Worker>, tasks: Vec<Task>) -> PlacementProblem {
        PlacementProblem { workers, tasks }
    }

    #[test]
    fn adjacent_half_open_intervals_can_reuse_the_same_worker() {
        let p = problem(
            vec![worker(1_000)],
            vec![
                task("first", 0, 10, 1_000),
                task("second", 10, 20, 1_000),
            ],
        );
        let report = solve_exact(&p, SolveLimits::default()).unwrap();
        assert_eq!(report.status, SolveStatus::Optimal);
        assert_eq!(report.feasible_assignments, 1);
        assert_eq!(report.frontier[0].peak_cpu_pressure_ppm, 1_000_000);
        assert_eq!(report.frontier[0].peak_memory_pressure_ppm, 125_000);
        assert_eq!(report.frontier[0].peak_gpu_pressure_ppm, 0);
    }

    #[test]
    fn overlapping_tasks_cannot_overcommit_a_worker() {
        let p = problem(
            vec![worker(1_000)],
            vec![
                task("first", 0, 10, 600),
                task("second", 9, 20, 600),
            ],
        );
        let report = solve_exact(&p, SolveLimits::default()).unwrap();
        assert_eq!(report.status, SolveStatus::Infeasible);
        assert!(report.frontier.is_empty());
    }

    #[test]
    fn nested_intervals_and_multiple_boundary_events_use_peak_concurrency() {
        let mut w = worker(3_000);
        w.memory_mib = 2_048;
        w.gpu_slots = 2;
        let mut a = task("outer-a", 0, 10, 1_000);
        a.memory_mib = 512;
        a.gpu_slots = 1;
        let mut b = task("nested", 2, 8, 1_500);
        b.memory_mib = 512;
        b.gpu_slots = 1;
        let mut c = task("outer-c", 8, 12, 2_000);
        c.memory_mib = 512;
        c.gpu_slots = 1;
        let report = solve_exact(&problem(vec![w], vec![a, b, c]), SolveLimits::default()).unwrap();
        assert_eq!(report.status, SolveStatus::Optimal);
        assert_eq!(report.feasible_assignments, 1);
        assert_eq!(report.frontier[0].peak_cpu_pressure_ppm, 1_000_000);
        assert_eq!(report.frontier[0].peak_memory_pressure_ppm, 500_000);
        assert_eq!(report.frontier[0].peak_gpu_pressure_ppm, 1_000_000);
    }

    #[test]
    fn cpu_and_memory_bottleneck_tradeoffs_are_kept_separate() {
        let mut cpu_heavy = worker(1_000);
        cpu_heavy.memory_mib = 2_000;
        cpu_heavy.gpu_slots = 0;

        let mut memory_heavy = worker(2_000);
        memory_heavy.worker_id = "worker-memory".into();
        memory_heavy.cpu_millis = 2_000;
        memory_heavy.memory_mib = 1_000;
        memory_heavy.gpu_slots = 0;

        let mut balanced_task = task("balanced", 0, 10, 1_000);
        balanced_task.memory_mib = 1_000;

        let report = solve_exact(
            &problem(vec![cpu_heavy, memory_heavy], vec![balanced_task]),
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
    fn region_permissions_remain_hard_during_temporal_placement() {
        let p = problem(
            vec![worker(2_000)],
            vec![Task {
                task_id: "restricted".into(),
                permitted_regions: vec!["region-other".into()],
                input_region: "region-other".into(),
                input_bytes: 0,
                cpu_millis: 100,
                memory_mib: 100,
                gpu_slots: 0,
                start_ms: 0,
                end_ms: 10,
            }],
        );
        let report = solve_exact(&p, SolveLimits::default()).unwrap();
        assert_eq!(report.status, SolveStatus::Infeasible);
    }

    #[test]
    fn reversed_and_zero_length_intervals_are_rejected() {
        for (start_ms, end_ms) in [(5, 5), (9, 3)] {
            let p = problem(vec![worker(1_000)], vec![task("invalid", start_ms, end_ms, 100)]);
            assert_eq!(
                solve_exact(&p, SolveLimits::default()),
                Err(SolveError::InvalidTaskInterval("invalid".into()))
            );
        }
    }

    #[test]
    fn exhausted_node_budget_never_claims_optimality() {
        let p = problem(
            vec![worker(1_000), {
                let mut w = worker(1_000);
                w.worker_id = "worker-b".into();
                w.region_id = "region-a".into();
                w.operator_id = "operator-b".into();
                w.failure_domain_id = "fd-b".into();
                w
            }],
            vec![task("first", 0, 10, 100), task("second", 10, 20, 100)],
        );
        let report = solve_exact(
            &p,
            SolveLimits { max_search_nodes: 1, max_frontier_size: 16 },
        ).unwrap();
        assert_eq!(report.status, SolveStatus::Incomplete);
    }

    #[test]
    fn reordered_inputs_produce_identical_frontier() {
        let mut second_worker = worker(2_000);
        second_worker.worker_id = "worker-b".into();
        second_worker.region_id = "region-b".into();
        second_worker.operator_id = "operator-b".into();
        second_worker.failure_domain_id = "fd-b".into();
        let p = problem(
            vec![worker(2_000), second_worker],
            vec![task("first", 0, 10, 100), task("second", 10, 20, 100)],
        );
        let expected = solve_exact(&p, SolveLimits::default()).unwrap();
        let mut reordered = p.clone();
        reordered.workers.reverse();
        reordered.tasks.reverse();
        assert_eq!(solve_exact(&reordered, SolveLimits::default()).unwrap(), expected);
    }

    #[test]
    fn search_frontier_limit_returns_incomplete() {
        let first_worker = worker(2_000);
        let mut second_worker = worker(2_000);
        second_worker.worker_id = "worker-b".into();
        second_worker.region_id = "region-b".into();
        second_worker.cpu_price_microusd_per_vcpu_hour = 500;

        let mut flexible_task = task("first", 0, 3_600_000, 1_000);
        flexible_task.permitted_regions = vec!["region-a".into(), "region-b".into()];
        flexible_task.input_bytes = 1_024;

        let p = problem(
            vec![first_worker, second_worker],
            vec![flexible_task],
        );
        let report = solve_exact(
            &p,
            SolveLimits { max_search_nodes: 100, max_frontier_size: 1 },
        ).unwrap();
        assert_eq!(report.status, SolveStatus::Incomplete);
    }

    #[test]
    fn time_window_duration_drives_estimated_cost() {
        let short = problem(vec![worker(1_000)], vec![task("short", 0, 3_600_000, 1_000)]);
        let long = problem(vec![worker(1_000)], vec![task("long", 0, 7_200_000, 1_000)]);
        let short_cost = solve_exact(&short, SolveLimits::default()).unwrap().frontier[0].estimated_cost_microusd;
        let long_cost = solve_exact(&long, SolveLimits::default()).unwrap().frontier[0].estimated_cost_microusd;
        assert!(long_cost > short_cost);
    }
}
