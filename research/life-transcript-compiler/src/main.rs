use std::cmp::Reverse;
use std::collections::HashMap;
use std::fmt::Write as _;

const SUBJECTS: usize = 512;
const ROUNDS: usize = 4;
const MIN_P05_CANDIDATES: usize = 16;
const SEED: u64 = 0x5EED_C0DE;

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
    const ALL: [Self; 3] = [Self::Exact, Self::Bucketed, Self::Padded];

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
    const ALL: [Self; 3] = [Self::Exact, Self::Coarse, Self::Omitted];

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
    for round in 0..ROUNDS {
        count_fields[round] = plan.rounds[round]
            .count
            .observable(subject.statement_counts[round]);
        index_fields[round] = plan.rounds[round]
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

fn generated_index_mask(subject: usize, round: usize) -> u16 {
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

fn generate_subjects() -> Vec<Subject> {
    (0..SUBJECTS)
        .map(|index| {
            let index_u64 = index as u64;
            let mut statement_counts = [0_u8; ROUNDS];
            let mut index_masks = [0_u16; ROUNDS];
            let mut task_answers = [0_u8; ROUNDS];
            for round in 0..ROUNDS {
                let round_u64 = round as u64;
                let count_seed = SEED
                    ^ index_u64.wrapping_mul(0xD6E8_FEB8_6659_FD93)
                    ^ round_u64.wrapping_mul(0xA076_1D64_78BD_642F);
                statement_counts[round] = 8 + (splitmix64(count_seed) % 24) as u8;
                index_masks[round] = generated_index_mask(index, round);
                task_answers[round] = ((index >> (round % 3)) & 1) as u8;
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

fn task_answers_preserved(subjects: &[Subject], plan: Plan) -> bool {
    subjects.iter().all(|subject| {
        compile_observer_view(subject, plan, false).task_answers == subject.task_answers
    })
}

fn policy_feasible(metrics: &Metrics) -> bool {
    metrics.p05_candidates >= MIN_P05_CANDIDATES
}

fn select_plan(plans: &[Plan], metrics: &[Metrics]) -> usize {
    (0..plans.len())
        .filter(|&index| policy_feasible(&metrics[index]))
        .min_by_key(|&index| {
            (
                plans[index].cost(),
                Reverse(metrics[index].p05_candidates),
                plans[index].label(),
            )
        })
        .expect("frozen plan space should contain a feasible plan")
}

fn is_on_frontier(index: usize, plans: &[Plan], metrics: &[Metrics]) -> bool {
    let cost = plans[index].cost();
    let support = metrics[index].p05_candidates;
    !(0..plans.len()).any(|other| {
        if index == other {
            return false;
        }
        let other_cost = plans[other].cost();
        let other_support = metrics[other].p05_candidates;
        other_cost <= cost
            && other_support >= support
            && (other_cost < cost || other_support > support)
    })
}

fn ppm(count: usize, denominator: usize) -> usize {
    count * 1_000_000 / denominator
}

fn render_report(subjects: &[Subject]) -> String {
    let plans = enumerate_plans();
    let metrics: Vec<Metrics> = plans
        .iter()
        .map(|&plan| evaluate(subjects, plan, false))
        .collect();
    let selected = select_plan(&plans, &metrics);
    let frontier: Vec<bool> = (0..plans.len())
        .map(|index| is_on_frontier(index, &plans, &metrics))
        .collect();
    let mut report = String::from(
        "plan_index,plan,rounds,total_cost_units,subjects,task_answer_equivalence,unique_candidate_count,unique_candidate_ppm,under_k_count,under_k_ppm,min_candidates,p05_candidates,p50_candidates,p95_candidates,max_candidates,frontier,feasible,selected\n",
    );
    for (index, plan) in plans.iter().enumerate() {
        let metric = &metrics[index];
        writeln!(
            report,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            index,
            plan.label(),
            ROUNDS,
            plan.cost(),
            subjects.len(),
            task_answers_preserved(subjects, *plan),
            metric.unique_candidate_count,
            ppm(metric.unique_candidate_count, subjects.len()),
            metric.under_k_count,
            ppm(metric.under_k_count, subjects.len()),
            metric.min_candidates,
            metric.p05_candidates,
            metric.p50_candidates,
            metric.p95_candidates,
            metric.max_candidates,
            frontier[index],
            policy_feasible(metric),
            index == selected,
        )
        .expect("writing to String cannot fail");
    }

    let stable_id_plan = Plan {
        rounds: [RoundProfile { count: CountMode::Exact, index: IndexMode::Exact }; ROUNDS],
    };
    let stable_id_metrics = evaluate(subjects, stable_id_plan, true);
    writeln!(
        report,
        "control,stable_id_negative_control,{},{},{},{},{},{},{},{},{},{},{},{},{},{},false,false",
        ROUNDS,
        stable_id_plan.cost(),
        subjects.len(),
        true,
        stable_id_metrics.unique_candidate_count,
        ppm(stable_id_metrics.unique_candidate_count, subjects.len()),
        stable_id_metrics.under_k_count,
        ppm(stable_id_metrics.under_k_count, subjects.len()),
        stable_id_metrics.min_candidates,
        stable_id_metrics.p05_candidates,
        stable_id_metrics.p50_candidates,
        stable_id_metrics.p95_candidates,
        stable_id_metrics.max_candidates,
        false,
    )
    .expect("writing to String cannot fail");
    report
}

fn main() {
    print!("{}", render_report(&generate_subjects()));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exactly_6561_four_round_plans_are_enumerated() {
        assert_eq!(enumerate_plans().len(), 6_561);
    }

    #[test]
    fn every_plan_preserves_the_exact_required_task_transcript() {
        let subjects = generate_subjects();
        for plan in enumerate_plans() {
            assert!(task_answers_preserved(&subjects, plan), "{}", plan.label());
        }
    }

    #[test]
    fn exact_metadata_baseline_fails_global_p05_floor() {
        let subjects = generate_subjects();
        let plan = Plan {
            rounds: [RoundProfile { count: CountMode::Exact, index: IndexMode::Exact }; ROUNDS],
        };
        let metrics = evaluate(&subjects, plan, false);
        assert!(metrics.p05_candidates < MIN_P05_CANDIDATES);
    }

    #[test]
    fn stable_identifier_positive_control_uniquely_identifies_subjects() {
        let subjects = generate_subjects();
        let plan = Plan {
            rounds: [ROUND_PROFILES[0]; ROUNDS],
        };
        let metrics = evaluate(&subjects, plan, true);
        assert_eq!(metrics.unique_candidate_count, SUBJECTS);
        assert_eq!(metrics.p05_candidates, 1);
    }

    #[test]
    fn selected_plan_matches_independent_golden_expectation() {
        let subjects = generate_subjects();
        let plans = enumerate_plans();
        let metrics: Vec<Metrics> = plans
            .iter()
            .map(|&plan| evaluate(&subjects, plan, false))
            .collect();
        let selected = select_plan(&plans, &metrics);
        let plan = plans[selected];
        assert_eq!(
            plan.rounds[0],
            RoundProfile { count: CountMode::Padded, index: IndexMode::Omitted }
        );
        assert_eq!(
            plan.rounds[1],
            RoundProfile { count: CountMode::Padded, index: IndexMode::Omitted }
        );
        assert_eq!(
            plan.rounds[2],
            RoundProfile { count: CountMode::Bucketed, index: IndexMode::Omitted }
        );
        assert_eq!(
            plan.rounds[3],
            RoundProfile { count: CountMode::Padded, index: IndexMode::Omitted }
        );
        assert_eq!(plan.cost(), 128);
        assert_eq!(metrics[selected].p05_candidates, 27);
        assert_eq!(metrics[selected].under_k_count, 0);
        assert_eq!(metrics[selected].unique_candidate_count, 0);
    }

    #[test]
    fn coarsening_a_channel_cannot_reduce_support_for_any_target() {
        let subjects = generate_subjects();
        let exact = Plan {
            rounds: [ROUND_PROFILES[0]; ROUNDS],
        };
        let mut count_bucketed = exact;
        count_bucketed.rounds[0].count = CountMode::Bucketed;
        let mut count_padded = count_bucketed;
        count_padded.rounds[0].count = CountMode::Padded;
        let mut index_coarse = exact;
        index_coarse.rounds[0].index = IndexMode::Coarse;
        let mut index_omitted = index_coarse;
        index_omitted.rounds[0].index = IndexMode::Omitted;

        for subject in &subjects {
            let exact_count = candidate_count_for_subject(subject, &subjects, exact, false);
            let bucket_count =
                candidate_count_for_subject(subject, &subjects, count_bucketed, false);
            let padded_count =
                candidate_count_for_subject(subject, &subjects, count_padded, false);
            assert!(exact_count <= bucket_count);
            assert!(bucket_count <= padded_count);

            let exact_index = candidate_count_for_subject(subject, &subjects, exact, false);
            let coarse_index =
                candidate_count_for_subject(subject, &subjects, index_coarse, false);
            let omitted_index =
                candidate_count_for_subject(subject, &subjects, index_omitted, false);
            assert!(exact_index <= coarse_index);
            assert!(coarse_index <= omitted_index);
        }
    }

    #[test]
    fn reported_frontier_matches_independent_pairwise_oracle() {
        let subjects = generate_subjects();
        let plans = enumerate_plans();
        let metrics: Vec<Metrics> = plans
            .iter()
            .map(|&plan| evaluate(&subjects, plan, false))
            .collect();
        for index in 0..plans.len() {
            let dominated = plans.iter().zip(&metrics).enumerate().any(
                |(other, (other_plan, other_metrics))| {
                    if other == index {
                        return false;
                    }
                    let cost_no_worse = other_plan.cost() <= plans[index].cost();
                    let support_no_worse =
                        other_metrics.p05_candidates >= metrics[index].p05_candidates;
                    let at_least_one_better =
                        other_plan.cost() < plans[index].cost()
                            || other_metrics.p05_candidates > metrics[index].p05_candidates;
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
        let subjects = generate_subjects();
        let first = render_report(&subjects);
        let second = render_report(&subjects);
        assert_eq!(first, second);
        let mut lines = first.lines();
        let header_columns = lines.next().expect("header exists").split(',').count();
        assert_eq!(header_columns, 18);
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
        assert_eq!(row_count, 6_562);
        assert_eq!(first.lines().count(), 6_563);
    }
}
