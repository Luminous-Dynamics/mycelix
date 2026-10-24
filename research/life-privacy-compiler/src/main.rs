use std::cmp::Reverse;
use std::fmt::Write as _;

const SUBJECTS: usize = 512;
const TASK_BITS: u8 = 3;
const MIN_P05_CANDIDATES: usize = 16;
const SEED: u64 = 0x5EED_C0DE;

#[derive(Clone, Debug, Eq, PartialEq)]
struct Subject {
    task_answers: u8,
    statement_count: u8,
    optional_index_mask: u16,
    stable_identifier: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CountMode {
    Exact,
    Bucketed,
    Padded,
}

impl CountMode {
    const ALL: [Self; 3] = [Self::Exact, Self::Bucketed, Self::Padded];

    fn label(self) -> &'static str {
        match self {
            Self::Exact => "exact",
            Self::Bucketed => "two_bucket",
            Self::Padded => "common_padding",
        }
    }

    fn cost(self) -> usize {
        match self {
            Self::Exact => 0,
            Self::Bucketed => 8,
            Self::Padded => 24,
        }
    }

    fn observable(self, subject: &Subject) -> u8 {
        match self {
            Self::Exact => subject.statement_count,
            Self::Bucketed => u8::from(subject.statement_count >= 20),
            Self::Padded => u8::MAX,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum IndexMode {
    Exact,
    Coarse,
    Omitted,
}

impl IndexMode {
    const ALL: [Self; 3] = [Self::Exact, Self::Coarse, Self::Omitted];

    fn label(self) -> &'static str {
        match self {
            Self::Exact => "exact_pattern",
            Self::Coarse => "coarse_class",
            Self::Omitted => "omitted",
        }
    }

    fn cost(self) -> usize {
        match self {
            Self::Exact => 0,
            Self::Coarse => 5,
            Self::Omitted => 12,
        }
    }

    fn observable(self, subject: &Subject) -> Option<u8> {
        match self {
            Self::Exact => Some(subject.optional_index_mask as u8),
            Self::Coarse => Some((subject.optional_index_mask % 4) as u8),
            Self::Omitted => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Strategy {
    count_mode: CountMode,
    index_mode: IndexMode,
    stable_identifier_visible: bool,
}

impl Strategy {
    fn label(self) -> &'static str {
        if self.stable_identifier_visible {
            "stable_id_negative_control"
        } else {
            match (self.count_mode, self.index_mode) {
                (CountMode::Exact, IndexMode::Exact) => "count_exact__index_exact",
                (CountMode::Exact, IndexMode::Coarse) => "count_exact__index_coarse",
                (CountMode::Exact, IndexMode::Omitted) => "count_exact__index_omitted",
                (CountMode::Bucketed, IndexMode::Exact) => "count_bucketed__index_exact",
                (CountMode::Bucketed, IndexMode::Coarse) => "count_bucketed__index_coarse",
                (CountMode::Bucketed, IndexMode::Omitted) => "count_bucketed__index_omitted",
                (CountMode::Padded, IndexMode::Exact) => "count_padded__index_exact",
                (CountMode::Padded, IndexMode::Coarse) => "count_padded__index_coarse",
                (CountMode::Padded, IndexMode::Omitted) => "count_padded__index_omitted",
            }
        }
    }

    fn overhead(self) -> usize {
        self.count_mode.cost() + self.index_mode.cost()
    }

    fn policy_eligible(self) -> bool {
        !self.stable_identifier_visible
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ObserverView {
    task_answers: u8,
    statement_count_field: Option<u8>,
    index_pattern_field: Option<u8>,
    stable_identifier_field: Option<u16>,
}

fn compile_observer_view(subject: &Subject, strategy: Strategy) -> ObserverView {
    ObserverView {
        task_answers: subject.task_answers,
        statement_count_field: Some(strategy.count_mode.observable(subject)),
        index_pattern_field: strategy.index_mode.observable(subject),
        stable_identifier_field: strategy
            .stable_identifier_visible
            .then_some(subject.stable_identifier),
    }
}

fn splitmix64(input: u64) -> u64 {
    let mut z = input.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn generated_index_mask(index: usize) -> u16 {
    let mut mask = 0_u16;
    let mut salt = 0_u64;
    while mask.count_ones() < 3 {
        let input = SEED
            .wrapping_add((index as u64).wrapping_mul(0xA076_1D64_78BD_642F))
            .wrapping_add(salt.wrapping_mul(0xE703_7ED1_A0B4_28DB));
        let bit = (splitmix64(input) % 8) as u32;
        mask |= 1_u16 << bit;
        salt += 1;
    }
    mask
}

fn generate_subjects() -> Vec<Subject> {
    (0..SUBJECTS)
        .map(|index| {
            let index_u64 = index as u64;
            let count_seed = SEED ^ index_u64.wrapping_mul(0xD6E8_FEB8_6659_FD93);
            Subject {
                task_answers: (index % (1 << TASK_BITS)) as u8,
                statement_count: 8 + (splitmix64(count_seed) % 24) as u8,
                optional_index_mask: generated_index_mask(index),
                stable_identifier: index as u16,
            }
        })
        .collect()
}

fn strategies() -> Vec<Strategy> {
    let mut all = Vec::with_capacity(CountMode::ALL.len() * IndexMode::ALL.len() + 1);
    for count_mode in CountMode::ALL {
        for index_mode in IndexMode::ALL {
            all.push(Strategy {
                count_mode,
                index_mode,
                stable_identifier_visible: false,
            });
        }
    }
    all.push(Strategy {
        count_mode: CountMode::Exact,
        index_mode: IndexMode::Exact,
        stable_identifier_visible: true,
    });
    all
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Metrics {
    unique_candidate_count: usize,
    under_k_count: usize,
    min_candidates: usize,
    p05_candidates: usize,
    p50_candidates: usize,
    p95_candidates: usize,
    max_candidates: usize,
}

fn percentile(sorted: &[usize], percentile: usize) -> usize {
    let rank = (percentile * sorted.len()).div_ceil(100).max(1);
    sorted[rank - 1]
}

fn observable_match(left: &Subject, right: &Subject, strategy: Strategy) -> bool {
    compile_observer_view(left, strategy) == compile_observer_view(right, strategy)
}

fn candidate_count(target: &Subject, subjects: &[Subject], strategy: Strategy) -> usize {
    subjects
        .iter()
        .filter(|candidate| observable_match(target, candidate, strategy))
        .count()
}

fn evaluate(subjects: &[Subject], strategy: Strategy) -> Metrics {
    let candidate_counts: Vec<usize> = subjects
        .iter()
        .map(|target| candidate_count(target, subjects, strategy))
        .collect();
    let mut sorted = candidate_counts.clone();
    sorted.sort_unstable();
    Metrics {
        unique_candidate_count: sorted.iter().filter(|&&count| count == 1).count(),
        under_k_count: sorted
            .iter()
            .filter(|&&count| count < MIN_P05_CANDIDATES)
            .count(),
        min_candidates: sorted[0],
        p05_candidates: percentile(&sorted, 5),
        p50_candidates: percentile(&sorted, 50),
        p95_candidates: percentile(&sorted, 95),
        max_candidates: *sorted.last().expect("synthetic population is nonempty"),
    }
}

fn policy_feasible(strategy: Strategy, metrics: &Metrics) -> bool {
    strategy.policy_eligible() && metrics.p05_candidates >= MIN_P05_CANDIDATES
}

fn choose_strategy(strategies: &[Strategy], metrics: &[Metrics]) -> usize {
    (0..strategies.len())
        .filter(|&index| policy_feasible(strategies[index], &metrics[index]))
        .min_by_key(|&index| {
            (
                strategies[index].overhead(),
                Reverse(metrics[index].p05_candidates),
                strategies[index].label(),
            )
        })
        .expect("the frozen policy should have at least one feasible strategy")
}

fn is_pareto_frontier(index: usize, strategies: &[Strategy], metrics: &[Metrics]) -> bool {
    let current = strategies[index];
    if !current.policy_eligible() {
        return false;
    }
    !(0..strategies.len()).any(|other_index| {
        if other_index == index || !strategies[other_index].policy_eligible() {
            return false;
        }
        let other_cost = strategies[other_index].overhead();
        let other_p05 = metrics[other_index].p05_candidates;
        let current_cost = current.overhead();
        let current_p05 = metrics[index].p05_candidates;
        other_cost <= current_cost
            && other_p05 >= current_p05
            && (other_cost < current_cost || other_p05 > current_p05)
    })
}

fn ppm(count: usize, denominator: usize) -> usize {
    count * 1_000_000 / denominator
}

fn render_report(subjects: &[Subject]) -> String {
    let strategies = strategies();
    let metrics: Vec<Metrics> = strategies
        .iter()
        .map(|&strategy| evaluate(subjects, strategy))
        .collect();
    let selected = choose_strategy(&strategies, &metrics);
    let mut report = String::from(
        "strategy,count_mode,index_mode,stable_identifier_visible,overhead_units,subjects,task_answer_equivalence,unique_candidate_count,unique_candidate_ppm,under_k_count,under_k_ppm,min_candidates,p05_candidates,p50_candidates,p95_candidates,max_candidates,policy_eligible,policy_feasible,pareto_frontier,selected\n",
    );
    for (index, strategy) in strategies.iter().enumerate() {
        let metric = &metrics[index];
        writeln!(
            report,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            strategy.label(),
            strategy.count_mode.label(),
            strategy.index_mode.label(),
            strategy.stable_identifier_visible,
            strategy.overhead(),
            subjects.len(),
            true,
            metric.unique_candidate_count,
            ppm(metric.unique_candidate_count, subjects.len()),
            metric.under_k_count,
            ppm(metric.under_k_count, subjects.len()),
            metric.min_candidates,
            metric.p05_candidates,
            metric.p50_candidates,
            metric.p95_candidates,
            metric.max_candidates,
            strategy.policy_eligible(),
            policy_feasible(*strategy, metric),
            is_pareto_frontier(index, &strategies, &metrics),
            index == selected,
        )
        .expect("writing to String cannot fail");
    }
    report
}

fn main() {
    print!("{}", render_report(&generate_subjects()));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_profile_preserves_exact_task_answers_for_all_subjects() {
        let subjects = generate_subjects();
        for strategy in strategies() {
            for subject in &subjects {
                let view = compile_observer_view(subject, strategy);
                assert_eq!(
                    view.task_answers, subject.task_answers,
                    "task answers changed under {}",
                    strategy.label()
                );
            }
        }
    }

    #[test]
    fn task_answer_signatures_are_balanced() {
        let subjects = generate_subjects();
        let mut counts = [0_usize; 1 << TASK_BITS];
        for subject in &subjects {
            counts[subject.task_answers as usize] += 1;
        }
        assert!(counts.iter().all(|&count| count == SUBJECTS / (1 << TASK_BITS)));
    }

    #[test]
    fn unmitigated_and_stable_id_controls_fail_policy() {
        let subjects = generate_subjects();
        let all = strategies();
        let baseline = all
            .iter()
            .find(|s| s.label() == "count_exact__index_exact")
            .expect("baseline exists");
        let baseline_metrics = evaluate(&subjects, *baseline);
        assert!(baseline_metrics.p05_candidates < MIN_P05_CANDIDATES);

        let stable_id = all
            .iter()
            .find(|s| s.stable_identifier_visible)
            .expect("stable-ID control exists");
        let stable_metrics = evaluate(&subjects, *stable_id);
        assert_eq!(stable_metrics.unique_candidate_count, SUBJECTS);
        assert_eq!(stable_metrics.p05_candidates, 1);
        assert!(!stable_id.policy_eligible());
        assert!(!policy_feasible(*stable_id, &stable_metrics));
    }

    #[test]
    fn compiler_selects_the_lowest_cost_feasible_profile() {
        let subjects = generate_subjects();
        let all = strategies();
        let metrics: Vec<Metrics> = all.iter().map(|&s| evaluate(&subjects, s)).collect();
        let selected_index = choose_strategy(&all, &metrics);
        assert_eq!(all[selected_index].label(), "count_bucketed__index_omitted");
        assert_eq!(all[selected_index].overhead(), 20);
        assert!(policy_feasible(all[selected_index], &metrics[selected_index]));
        assert_eq!(metrics[selected_index].p05_candidates, 26);

        for (index, strategy) in all.iter().enumerate() {
            if policy_feasible(*strategy, &metrics[index]) {
                assert!(strategy.overhead() >= all[selected_index].overhead());
            }
        }
    }

    #[test]
    fn adding_observer_fields_cannot_increase_candidate_sets() {
        let subjects = generate_subjects();
        for subject in &subjects {
            let bucketed_omitted = candidate_count(
                subject,
                &subjects,
                Strategy {
                    count_mode: CountMode::Bucketed,
                    index_mode: IndexMode::Omitted,
                    stable_identifier_visible: false,
                },
            );
            let bucketed_exact = candidate_count(
                subject,
                &subjects,
                Strategy {
                    count_mode: CountMode::Bucketed,
                    index_mode: IndexMode::Exact,
                    stable_identifier_visible: false,
                },
            );
            assert!(bucketed_exact <= bucketed_omitted);

            let exact_omitted = candidate_count(
                subject,
                &subjects,
                Strategy {
                    count_mode: CountMode::Exact,
                    index_mode: IndexMode::Omitted,
                    stable_identifier_visible: false,
                },
            );
            let exact_exact = candidate_count(
                subject,
                &subjects,
                Strategy {
                    count_mode: CountMode::Exact,
                    index_mode: IndexMode::Exact,
                    stable_identifier_visible: false,
                },
            );
            assert!(exact_exact <= exact_omitted);
        }
    }

    #[test]
    fn pairwise_frontier_matches_feasible_strategy_dominance() {
        let subjects = generate_subjects();
        let all = strategies();
        let metrics: Vec<Metrics> = all.iter().map(|&s| evaluate(&subjects, s)).collect();
        for index in 0..all.len() {
            let expected = is_pareto_frontier(index, &all, &metrics);
            if expected {
                assert!(all[index].policy_eligible());
                for other in 0..all.len() {
                    if index == other || !all[other].policy_eligible() {
                        continue;
                    }
                    let no_worse_cost = all[other].overhead() <= all[index].overhead();
                    let no_worse_privacy = metrics[other].p05_candidates >= metrics[index].p05_candidates;
                    let strict = all[other].overhead() < all[index].overhead()
                        || metrics[other].p05_candidates > metrics[index].p05_candidates;
                    assert!(!(no_worse_cost && no_worse_privacy && strict));
                }
            }
        }
        let selected = choose_strategy(&all, &metrics);
        assert!(is_pareto_frontier(selected, &all, &metrics));
    }

    #[test]
    fn report_is_rectangular_deterministic_and_contains_one_selection() {
        let subjects = generate_subjects();
        let first = render_report(&subjects);
        let second = render_report(&subjects);
        assert_eq!(first, second);
        let mut lines = first.lines();
        let header_columns = lines.next().expect("header exists").split(',').count();
        assert_eq!(header_columns, 20);
        let mut selected_count = 0;
        for line in lines {
            assert_eq!(line.split(',').count(), header_columns);
            if line.ends_with(",true") {
                selected_count += 1;
            }
        }
        assert_eq!(selected_count, 1);
        assert_eq!(first.lines().count(), strategies().len() + 1);
    }

    #[test]
    fn percentile_summary_is_monotone() {
        let metrics = evaluate(
            &generate_subjects(),
            Strategy {
                count_mode: CountMode::Bucketed,
                index_mode: IndexMode::Omitted,
                stable_identifier_visible: false,
            },
        );
        assert!(metrics.min_candidates <= metrics.p05_candidates);
        assert!(metrics.p05_candidates <= metrics.p50_candidates);
        assert!(metrics.p50_candidates <= metrics.p95_candidates);
        assert!(metrics.p95_candidates <= metrics.max_candidates);
    }
}
