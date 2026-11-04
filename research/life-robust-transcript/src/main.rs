use std::cmp::Reverse;
use std::collections::HashMap;
use std::fmt::Write as _;

const SUBJECTS: usize = 512;
const ROUNDS: usize = 4;
const MIN_P05_CANDIDATES: usize = 16;
const SEED: u64 = 0x5EED_C0DE;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Scenario {
    Reference,
    Shifted,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Subject {
    task_answers: [u8; ROUNDS],
    statement_counts: [u8; ROUNDS],
    index_masks: [u16; ROUNDS],
    stable_identifier: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
enum CountMode {
    Exact,
    Bucketed,
    Padded,
}

impl CountMode {
    fn label(self) -> &'static str {
        match self {
            Self::Exact => "exact",
            Self::Bucketed => "bucketed",
            Self::Padded => "padded",
        }
    }

    fn cost(self) -> usize {
        match self {
            Self::Exact => 0,
            Self::Bucketed => 8,
            Self::Padded => 24,
        }
    }

    fn observable(self, count: u8) -> u8 {
        match self {
            Self::Exact => count,
            Self::Bucketed => u8::from(count >= 20),
            Self::Padded => u8::MAX,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
enum IndexMode {
    Exact,
    Coarse,
    Omitted,
}

impl IndexMode {
    fn label(self) -> &'static str {
        match self {
            Self::Exact => "exact",
            Self::Coarse => "coarse",
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

    fn observable(self, mask: u16) -> Option<u8> {
        match self {
            Self::Exact => Some(mask as u8),
            Self::Coarse => Some(((mask & 0x0f).count_ones() % 2) as u8),
            Self::Omitted => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
struct RoundProfile {
    count: CountMode,
    index: IndexMode,
}

impl RoundProfile {
    fn label(self) -> String {
        format!("{}+{}", self.count.label(), self.index.label())
    }

    fn cost(self) -> usize {
        self.count.cost() + self.index.cost()
    }
}

const ROUND_PROFILES: [RoundProfile; 9] = [
    RoundProfile { count: CountMode::Exact, index: IndexMode::Exact },
    RoundProfile { count: CountMode::Exact, index: IndexMode::Coarse },
    RoundProfile { count: CountMode::Exact, index: IndexMode::Omitted },
    RoundProfile { count: CountMode::Bucketed, index: IndexMode::Exact },
    RoundProfile { count: CountMode::Bucketed, index: IndexMode::Coarse },
    RoundProfile { count: CountMode::Bucketed, index: IndexMode::Omitted },
    RoundProfile { count: CountMode::Padded, index: IndexMode::Exact },
    RoundProfile { count: CountMode::Padded, index: IndexMode::Coarse },
    RoundProfile { count: CountMode::Padded, index: IndexMode::Omitted },
];

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
struct Plan {
    rounds: [RoundProfile; ROUNDS],
}

impl Plan {
    fn cost(self) -> usize {
        self.rounds.iter().map(|profile| profile.cost()).sum()
    }

    fn label(self) -> String {
        self.rounds
            .iter()
            .map(|profile| profile.label())
            .collect::<Vec<_>>()
            .join("|")
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
struct ObserverView {
    task_answers: [u8; ROUNDS],
    count_fields: [u8; ROUNDS],
    index_fields: [Option<u8>; ROUNDS],
    stable_identifier: Option<u16>,
}

fn compile_observer_view(
    subject: &Subject,
    plan: Plan,
    stable_identifier_visible: bool,
) -> ObserverView {
    let mut count_fields = [0_u8; ROUNDS];
    let mut index_fields = [None; ROUNDS];
    for (round, (count_field, index_field)) in count_fields
        .iter_mut()
        .zip(index_fields.iter_mut())
        .enumerate()
    {
        *count_field = plan.rounds[round]
            .count
            .observable(subject.statement_counts[round]);
        *index_field = plan.rounds[round]
            .index
            .observable(subject.index_masks[round]);
    }
    ObserverView {
        task_answers: subject.task_answers,
        count_fields,
        index_fields,
        stable_identifier: stable_identifier_visible.then_some(subject.stable_identifier),
    }
}

fn splitmix64(input: u64) -> u64 {
    let mut value = input.wrapping_add(0x9E37_79B9_7F4A_7C15);
    value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    value ^ (value >> 31)
}

fn generated_reference_mask(subject: usize, round: usize) -> u16 {
    let mut mask = 0_u16;
    let mut salt = 0_u64;
    while mask.count_ones() < 3 {
        let input = SEED
            .wrapping_add((subject as u64).wrapping_mul(0xA076_1D64_78BD_642F))
            .wrapping_add((round as u64).wrapping_mul(0xE703_7ED1_A0B4_28DB))
            .wrapping_add(salt.wrapping_mul(0x8EBC_6AF0_9C88_C6E3));
        let bit = (splitmix64(input) % 8) as u32;
        mask |= 1_u16 << bit;
        salt += 1;
    }
    mask
}

fn index_masks_by_parity(parity: u32) -> Vec<u16> {
    (0_u16..256)
        .filter(|mask| {
            let value = *mask;
            value.count_ones() == 3 && ((value & 0x0f).count_ones() % 2) == parity
        })
        .collect()
}

fn generate_subjects(scenario: Scenario) -> Vec<Subject> {
    let parity_pools = [index_masks_by_parity(0), index_masks_by_parity(1)];
    (0..SUBJECTS)
        .map(|index| {
            let index_u64 = index as u64;
            let task_signature = index % 8;
            let cohort_position = index / 8;
            let mut statement_counts = [0_u8; ROUNDS];
            let mut index_masks = [0_u16; ROUNDS];
            let mut task_answers = [0_u8; ROUNDS];

            for (round, ((statement_count, index_mask), task_answer)) in statement_counts
                .iter_mut()
                .zip(index_masks.iter_mut())
                .zip(task_answers.iter_mut())
                .enumerate()
            {
                let round_u64 = round as u64;
                let count_seed = SEED
                    ^ index_u64.wrapping_mul(0xD6E8_FEB8_6659_FD93)
                    ^ round_u64.wrapping_mul(0xA076_1D64_78BD_642F);
                match scenario {
                    Scenario::Reference => {
                        *statement_count = 8 + (splitmix64(count_seed) % 24) as u8;
                        *index_mask = generated_reference_mask(index, round);
                    }
                    Scenario::Shifted => {
                        let shifted_low = cohort_position < 12;
                        let offset = round * 3 + task_signature;
                        *statement_count = if shifted_low {
                            8 + ((cohort_position + offset) % 12) as u8
                        } else {
                            20 + (((cohort_position - 12) + offset) % 12) as u8
                        };
                        let parity = if shifted_low { 0 } else { 1 };
                        let pool = &parity_pools[parity];
                        let mask_seed = SEED
                            ^ index_u64.wrapping_mul(0xD6E8_FEB8_6659_FD93)
                            ^ round_u64.wrapping_mul(0xA076_1D64_78BD_642F);
                        let selection = (splitmix64(mask_seed) % pool.len() as u64) as usize;
                        *index_mask = pool[selection];
                    }
                }
                *task_answer = ((index >> (round % 3)) & 1) as u8;
            }
            Subject {
                task_answers,
                statement_counts,
                index_masks,
                stable_identifier: index as u16,
            }
        })
        .collect()
}

fn enumerate_plans() -> Vec<Plan> {
    let mut plans = Vec::with_capacity(9_usize.pow(ROUNDS as u32));
    for first in ROUND_PROFILES {
        for second in ROUND_PROFILES {
            for third in ROUND_PROFILES {
                for fourth in ROUND_PROFILES {
                    plans.push(Plan { rounds: [first, second, third, fourth] });
                }
            }
        }
    }
    plans
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

#[cfg(test)]
fn candidate_count_for_subject(
    target: &Subject,
    subjects: &[Subject],
    plan: Plan,
    stable_identifier_visible: bool,
) -> usize {
    let target_view = compile_observer_view(target, plan, stable_identifier_visible);
    subjects
        .iter()
        .filter(|candidate| {
            compile_observer_view(candidate, plan, stable_identifier_visible) == target_view
        })
        .count()
}

fn evaluate(subjects: &[Subject], plan: Plan, stable_identifier_visible: bool) -> Metrics {
    let views: Vec<ObserverView> = subjects
        .iter()
        .map(|subject| compile_observer_view(subject, plan, stable_identifier_visible))
        .collect();
    let mut counts: HashMap<ObserverView, usize> = HashMap::new();
    for view in &views {
        *counts.entry(*view).or_insert(0) += 1;
    }
    let mut candidate_counts: Vec<usize> =
        views.iter().map(|view| counts[view]).collect();
    candidate_counts.sort_unstable();
    Metrics {
        unique_candidate_count: candidate_counts
            .iter()
            .filter(|&&count| count == 1)
            .count(),
        under_k_count: candidate_counts
            .iter()
            .filter(|&&count| count < MIN_P05_CANDIDATES)
            .count(),
        min_candidates: candidate_counts[0],
        p05_candidates: percentile(&candidate_counts, 5),
        p50_candidates: percentile(&candidate_counts, 50),
        p95_candidates: percentile(&candidate_counts, 95),
        max_candidates: *candidate_counts.last().expect("population is nonempty"),
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct RobustMetrics {
    reference: Metrics,
    shifted: Metrics,
    worst_p05: usize,
}

impl RobustMetrics {
    fn new(reference: Metrics, shifted: Metrics) -> Self {
        let worst_p05 = reference.p05_candidates.min(shifted.p05_candidates);
        Self { reference, shifted, worst_p05 }
    }

    fn feasible(&self) -> bool {
        self.reference.p05_candidates >= MIN_P05_CANDIDATES
            && self.shifted.p05_candidates >= MIN_P05_CANDIDATES
    }
}

fn task_answers_preserved(scenarios: &[Vec<Subject>; 2], plan: Plan) -> bool {
    scenarios.iter().all(|population| {
        population.iter().all(|subject| {
            compile_observer_view(subject, plan, false).task_answers == subject.task_answers
        })
    })
}

fn evaluate_robust(scenarios: &[Vec<Subject>; 2], plan: Plan) -> RobustMetrics {
    RobustMetrics::new(
        evaluate(&scenarios[0], plan, false),
        evaluate(&scenarios[1], plan, false),
    )
}

fn select_plan(plans: &[Plan], metrics: &[RobustMetrics]) -> usize {
    plans
        .iter()
        .zip(metrics)
        .enumerate()
        .filter(|(_, (_, metric))| metric.feasible())
        .min_by_key(|(_, (plan, metric))| {
            (
                plan.cost(),
                Reverse(metric.worst_p05),
                plan.label(),
            )
        })
        .map(|(index, _)| index)
        .expect("frozen uncertainty set should contain a feasible plan")
}

fn is_on_frontier(index: usize, plans: &[Plan], metrics: &[RobustMetrics]) -> bool {
    let cost = plans[index].cost();
    let support = metrics[index].worst_p05;
    !plans.iter().zip(metrics).enumerate().any(
        |(other, (other_plan, other_metrics))| {
            if index == other {
                return false;
            }
            let other_cost = other_plan.cost();
            let other_support = other_metrics.worst_p05;
            other_cost <= cost
                && other_support >= support
                && (other_cost < cost || other_support > support)
        },
    )
}

fn ppm(count: usize, denominator: usize) -> usize {
    count * 1_000_000 / denominator
}

fn render_report(scenarios: &[Vec<Subject>; 2]) -> String {
    let plans = enumerate_plans();
    let metrics: Vec<RobustMetrics> = plans
        .iter()
        .map(|&plan| evaluate_robust(scenarios, plan))
        .collect();
    let selected = select_plan(&plans, &metrics);
    let frontier: Vec<bool> = (0..plans.len())
        .map(|index| is_on_frontier(index, &plans, &metrics))
        .collect();

    let mut report = String::from(
        "plan_index,plan,total_cost_units,subjects,task_answer_equivalence,reference_unique_count,reference_unique_ppm,reference_under_k_count,reference_under_k_ppm,reference_min,reference_p05,reference_p50,reference_p95,reference_max,shift_unique_count,shift_unique_ppm,shift_under_k_count,shift_under_k_ppm,shift_min,shift_p05,shift_p50,shift_p95,shift_max,worst_p05,robust_feasible,frontier,selected\n",
    );

    for (index, plan) in plans.iter().enumerate() {
        let metric = &metrics[index];
        let reference = &metric.reference;
        let shifted = &metric.shifted;
        writeln!(
            report,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            index,
            plan.label(),
            plan.cost(),
            SUBJECTS,
            task_answers_preserved(scenarios, *plan),
            reference.unique_candidate_count,
            ppm(reference.unique_candidate_count, SUBJECTS),
            reference.under_k_count,
            ppm(reference.under_k_count, SUBJECTS),
            reference.min_candidates,
            reference.p05_candidates,
            reference.p50_candidates,
            reference.p95_candidates,
            reference.max_candidates,
            shifted.unique_candidate_count,
            ppm(shifted.unique_candidate_count, SUBJECTS),
            shifted.under_k_count,
            ppm(shifted.under_k_count, SUBJECTS),
            shifted.min_candidates,
            shifted.p05_candidates,
            shifted.p50_candidates,
            shifted.p95_candidates,
            shifted.max_candidates,
            metric.worst_p05,
            metric.feasible(),
            frontier[index],
            index == selected,
        )
        .expect("writing to String cannot fail");
    }
    report
}

fn main() {
    let scenarios = [
        generate_subjects(Scenario::Reference),
        generate_subjects(Scenario::Shifted),
    ];
    print!("{}", render_report(&scenarios));
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenarios() -> [Vec<Subject>; 2] {
        [
            generate_subjects(Scenario::Reference),
            generate_subjects(Scenario::Shifted),
        ]
    }

    fn single_population_optimum() -> Plan {
        Plan {
            rounds: [
                RoundProfile { count: CountMode::Padded, index: IndexMode::Omitted },
                RoundProfile { count: CountMode::Padded, index: IndexMode::Omitted },
                RoundProfile { count: CountMode::Bucketed, index: IndexMode::Omitted },
                RoundProfile { count: CountMode::Padded, index: IndexMode::Omitted },
            ],
        }
    }

    #[test]
    fn exactly_6561_four_round_plans_are_enumerated() {
        assert_eq!(enumerate_plans().len(), 6_561);
    }

    #[test]
    fn task_transcripts_match_across_reference_and_shifted_scenarios() {
        let populations = scenarios();
        for (reference, shifted) in populations[0].iter().zip(&populations[1]) {
            assert_eq!(reference.task_answers, shifted.task_answers);
        }
        let mut counts: HashMap<[u8; ROUNDS], usize> = HashMap::new();
        for subject in &populations[0] {
            *counts.entry(subject.task_answers).or_insert(0) += 1;
        }
        assert_eq!(counts.len(), 8);
        assert!(counts.values().all(|&count| count == 64));
    }

    #[test]
    fn every_plan_preserves_answers_under_both_scenarios() {
        let populations = scenarios();
        for plan in enumerate_plans() {
            assert!(task_answers_preserved(&populations, plan), "{}", plan.label());
        }
    }

    #[test]
    fn reference_optimum_fails_the_shifted_global_privacy_floor() {
        let populations = scenarios();
        let plan = single_population_optimum();
        let reference = evaluate(&populations[0], plan, false);
        let shifted = evaluate(&populations[1], plan, false);
        assert_eq!(reference.p05_candidates, 27);
        assert_eq!(shifted.p05_candidates, 12);
        assert_eq!(shifted.under_k_count, 96);
        assert!(reference.p05_candidates >= MIN_P05_CANDIDATES);
        assert!(shifted.p05_candidates < MIN_P05_CANDIDATES);
    }

    #[test]
    fn robust_compiler_selects_padding_and_omission_in_every_round() {
        let populations = scenarios();
        let plans = enumerate_plans();
        let metrics: Vec<RobustMetrics> = plans
            .iter()
            .map(|&plan| evaluate_robust(&populations, plan))
            .collect();
        let feasible_count = metrics.iter().filter(|metric| metric.feasible()).count();
        assert_eq!(feasible_count, 1);
        let selected = select_plan(&plans, &metrics);
        assert_eq!(
            plans[selected],
            Plan {
                rounds: [RoundProfile { count: CountMode::Padded, index: IndexMode::Omitted }; ROUNDS],
            }
        );
        assert_eq!(plans[selected].cost(), 144);
        assert_eq!(metrics[selected].reference.p05_candidates, 64);
        assert_eq!(metrics[selected].shifted.p05_candidates, 64);
        assert_eq!(metrics[selected].reference.under_k_count, 0);
        assert_eq!(metrics[selected].shifted.under_k_count, 0);
    }

    #[test]
    fn stable_identifier_positive_control_uniquely_identifies_in_both_scenarios() {
        let populations = scenarios();
        let exact_plan = Plan {
            rounds: [ROUND_PROFILES_EXACT; ROUNDS],
        };
        let reference = evaluate(&populations[0], exact_plan, true);
        let shifted = evaluate(&populations[1], exact_plan, true);
        assert_eq!(reference.unique_candidate_count, SUBJECTS);
        assert_eq!(shifted.unique_candidate_count, SUBJECTS);
    }

    const ROUND_PROFILES_EXACT: RoundProfile =
        RoundProfile { count: CountMode::Exact, index: IndexMode::Exact };

    #[test]
    fn coarsening_each_channel_cannot_reduce_support_for_any_target() {
        let populations = scenarios();
        let exact = Plan { rounds: [ROUND_PROFILES_EXACT; ROUNDS] };
        let mut count_bucketed = exact;
        count_bucketed.rounds[0].count = CountMode::Bucketed;
        let mut count_padded = exact;
        count_padded.rounds[0].count = CountMode::Padded;
        let mut index_coarse = exact;
        index_coarse.rounds[0].index = IndexMode::Coarse;
        let mut index_omitted = exact;
        index_omitted.rounds[0].index = IndexMode::Omitted;

        for population in &populations {
            for subject in population {
                let exact_count =
                    candidate_count_for_subject(subject, population, exact, false);
                let bucket_count =
                    candidate_count_for_subject(subject, population, count_bucketed, false);
                let padded_count =
                    candidate_count_for_subject(subject, population, count_padded, false);
                assert!(exact_count <= bucket_count);
                assert!(bucket_count <= padded_count);

                let exact_index =
                    candidate_count_for_subject(subject, population, exact, false);
                let coarse_index =
                    candidate_count_for_subject(subject, population, index_coarse, false);
                let omitted_index =
                    candidate_count_for_subject(subject, population, index_omitted, false);
                assert!(exact_index <= coarse_index);
                assert!(coarse_index <= omitted_index);
            }
        }
    }

    #[test]
    fn reported_frontier_matches_independent_pairwise_oracle() {
        let populations = scenarios();
        let plans = enumerate_plans();
        let metrics: Vec<RobustMetrics> = plans
            .iter()
            .map(|&plan| evaluate_robust(&populations, plan))
            .collect();
        for (index, (current_plan, current_metrics)) in plans.iter().zip(&metrics).enumerate() {
            let dominated = plans.iter().zip(&metrics).enumerate().any(
                |(other, (other_plan, other_metrics))| {
                    if other == index {
                        return false;
                    }
                    let cost_no_worse = other_plan.cost() <= current_plan.cost();
                    let support_no_worse =
                        other_metrics.worst_p05 >= current_metrics.worst_p05;
                    let at_least_one_better =
                        other_plan.cost() < current_plan.cost()
                            || other_metrics.worst_p05 > current_metrics.worst_p05;
                    cost_no_worse && support_no_worse && at_least_one_better
                },
            );
            assert_eq!(!dominated, is_on_frontier(index, &plans, &metrics));
        }
        let selected = select_plan(&plans, &metrics);
        assert!(is_on_frontier(selected, &plans, &metrics));
    }

    #[test]
    fn report_is_rectangular_deterministic_and_contains_one_selection() {
        let populations = scenarios();
        let first = render_report(&populations);
        let second = render_report(&populations);
        assert_eq!(first, second);
        let mut lines = first.lines();
        let header_columns = lines.next().expect("header exists").split(',').count();
        assert_eq!(header_columns, 27);
        let mut selected_count = 0;
        let mut row_count = 0;
        for line in lines {
            assert_eq!(line.split(',').count(), header_columns);
            if line.split(',').last() == Some("true") {
                selected_count += 1;
            }
            row_count += 1;
        }
        assert_eq!(selected_count, 1);
        assert_eq!(row_count, 6_561);
        assert_eq!(first.lines().count(), 6_562);
    }
}
