use std::fmt::Write as _;

const POPULATION_SIZE: usize = 512;
const FEATURE_COUNT: usize = 18;
const MAX_ADAPTIVE_QUERIES: usize = 6;
const MILLION: u64 = 1_000_000;
const REFERENCE_SEED: u64 = 0x4c49_4645_5155_4552;
const INDEPENDENT_SEED: u64 = 0x4c49_4645_5155_4553;

#[derive(Clone, Debug, PartialEq, Eq)]
struct Person {
    age_band: u8,
    region: u8,
    income_band: u8,
    sector: u8,
    family_size: u8,
    debt_risk: bool,
    homeowner: bool,
    public_transit: bool,
}

impl Person {
    fn features(&self) -> [bool; FEATURE_COUNT] {
        [
            self.age_band > 0,
            self.region < 4,
            self.age_band >= 2,
            self.age_band >= 4,
            self.age_band >= 6,
            self.income_band >= 2,
            self.income_band >= 4,
            self.sector == 0,
            self.sector == 1,
            self.sector == 2,
            self.family_size >= 3,
            self.family_size >= 5,
            self.debt_risk,
            self.homeowner,
            self.public_transit,
            self.income_band <= 1,
            self.family_size >= 2,
            self.region >= 4,
        ]
    }

    fn eligible(&self) -> bool {
        self.age_band > 0 && self.region < 4
    }

    fn high_income(&self) -> bool {
        self.income_band >= 4
    }
}

fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
    let mut value = *state;
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

fn synthetic_population(seed: u64) -> Vec<Person> {
    (0..POPULATION_SIZE)
        .map(|id| {
            let mut state = seed ^ (id as u64).wrapping_mul(0xd6e8_feb8_6659_fd93);
            let age_band = (splitmix64(&mut state) % 8) as u8;
            let region = (splitmix64(&mut state) % 8) as u8;
            let sector = (splitmix64(&mut state) % 6) as u8;
            let income_shift = u8::from(age_band >= 4);
            let income_band = (((splitmix64(&mut state) % 6) as u8) + income_shift) % 6;
            let family_size = 1 + (splitmix64(&mut state) % 6) as u8;
            let debt_risk = income_band <= 1 || splitmix64(&mut state).is_multiple_of(5);
            let homeowner = age_band >= 3
                && income_band >= 2
                && !splitmix64(&mut state).is_multiple_of(4);
            let public_transit = region < 4 && splitmix64(&mut state).is_multiple_of(2);

            Person {
                age_band,
                region,
                income_band,
                sector,
                family_size,
                debt_risk,
                homeowner,
                public_transit,
            }
        })
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ShiftProfile {
    MatchedControl,
    IndependentSeed,
    DebtCorrelationReversed,
    IncomeMarginalShift,
    RareTail,
}

impl ShiftProfile {
    const ALL: [Self; 5] = [
        Self::MatchedControl,
        Self::IndependentSeed,
        Self::DebtCorrelationReversed,
        Self::IncomeMarginalShift,
        Self::RareTail,
    ];

    fn name(self) -> &'static str {
        match self {
            Self::MatchedControl => "matched_control",
            Self::IndependentSeed => "independent_seed",
            Self::DebtCorrelationReversed => "debt_correlation_reversed",
            Self::IncomeMarginalShift => "income_marginal_shift",
            Self::RareTail => "rare_tail",
        }
    }

    fn apply(self, reference: &[Person]) -> Vec<Person> {
        match self {
            Self::MatchedControl => reference.to_vec(),
            Self::IndependentSeed => synthetic_population(INDEPENDENT_SEED),
            Self::DebtCorrelationReversed => reference
                .iter()
                .cloned()
                .map(|mut person| {
                    person.debt_risk = person.income_band >= 4;
                    person
                })
                .collect(),
            Self::IncomeMarginalShift => reference
                .iter()
                .cloned()
                .map(|mut person| {
                    person.income_band = (person.income_band + 2) % 6;
                    person
                })
                .collect(),
            Self::RareTail => reference
                .iter()
                .enumerate()
                .map(|(index, person)| {
                    let mut shifted = person.clone();
                    if index % 29 == 0 {
                        shifted.sector = 5;
                        shifted.family_size = 6;
                        shifted.income_band = 5;
                    }
                    shifted
                })
                .collect(),
        }
    }
}

fn feature_rows(people: &[Person]) -> Vec<[bool; FEATURE_COUNT]> {
    people.iter().map(Person::features).collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct QueryPolicy {
    name: &'static str,
    min_branch_size: usize,
    max_sensitive_uplift_ppm: u64,
}

impl QueryPolicy {
    const ALL: [Self; 3] = [
        Self {
            name: "unrestricted",
            min_branch_size: 1,
            max_sensitive_uplift_ppm: MILLION,
        },
        Self {
            name: "branch_k32",
            min_branch_size: 32,
            max_sensitive_uplift_ppm: MILLION,
        },
        Self {
            name: "dual_k32_uplift100k",
            min_branch_size: 32,
            max_sensitive_uplift_ppm: 100_000,
        },
    ];

    fn is_restrictive(self) -> bool {
        self.min_branch_size > 1 || self.max_sensitive_uplift_ppm < MILLION
    }
}

fn initial_cohort(people: &[Person], target: &Person) -> Vec<usize> {
    people
        .iter()
        .enumerate()
        .filter_map(|(candidate_id, candidate)| {
            (candidate.eligible() == target.eligible()).then_some(candidate_id)
        })
        .collect()
}

fn choose_query(
    rows: &[[bool; FEATURE_COUNT]],
    candidates: &[usize],
    asked: &[bool; FEATURE_COUNT],
) -> Option<usize> {
    let mut best_feature = None;
    let mut best_minority_count = 0;

    for (feature, already_asked) in asked.iter().enumerate() {
        if *already_asked {
            continue;
        }

        let yes_count = candidates
            .iter()
            .filter(|candidate_id| rows[**candidate_id][feature])
            .count();
        let no_count = candidates.len() - yes_count;
        let minority_count = yes_count.min(no_count);

        if minority_count > best_minority_count {
            best_minority_count = minority_count;
            best_feature = Some(feature);
        }
    }

    best_feature
}

fn split_candidates(
    rows: &[[bool; FEATURE_COUNT]],
    candidates: &[usize],
    feature: usize,
) -> (Vec<usize>, Vec<usize>) {
    let mut yes = Vec::new();
    let mut no = Vec::new();

    for candidate_id in candidates.iter().copied() {
        if rows[candidate_id][feature] {
            yes.push(candidate_id);
        } else {
            no.push(candidate_id);
        }
    }

    (yes, no)
}

fn sensitive_confidence_ppm(people: &[Person], candidates: &[usize]) -> (u64, u64) {
    if candidates.is_empty() {
        return (MILLION, MILLION);
    }

    let high_income = candidates
        .iter()
        .filter(|candidate_id| people[**candidate_id].high_income())
        .count();
    let debt_risk = candidates
        .iter()
        .filter(|candidate_id| people[**candidate_id].debt_risk)
        .count();
    let total = candidates.len() as u64;

    let high_income_confidence =
        high_income.max(candidates.len() - high_income) as u64 * MILLION / total;
    let debt_risk_confidence =
        debt_risk.max(candidates.len() - debt_risk) as u64 * MILLION / total;
    (high_income_confidence, debt_risk_confidence)
}

// The admission function deliberately receives no target ID or target answer.
// It evaluates both possible answer branches from the prior transcript only.
fn admitted_partition(
    reference: &[Person],
    reference_rows: &[[bool; FEATURE_COUNT]],
    candidates: &[usize],
    feature: usize,
    policy: QueryPolicy,
) -> Option<(Vec<usize>, Vec<usize>)> {
    let (yes, no) = split_candidates(reference_rows, candidates, feature);
    if yes.len() < policy.min_branch_size || no.len() < policy.min_branch_size {
        return None;
    }

    let before = sensitive_confidence_ppm(reference, candidates);
    for branch in [&yes, &no] {
        let after = sensitive_confidence_ppm(reference, branch);
        if after.0.saturating_sub(before.0) > policy.max_sensitive_uplift_ppm
            || after.1.saturating_sub(before.1) > policy.max_sensitive_uplift_ppm
        {
            return None;
        }
    }

    Some((yes, no))
}

#[derive(Clone, Debug)]
struct TargetOutcome {
    attacker_candidates: Vec<usize>,
    accepted_queries: usize,
    stopped_by_guard: bool,
    max_income_confidence_uplift_ppm: u64,
    max_debt_confidence_uplift_ppm: u64,
}

fn evaluate(
    reference: &[Person],
    attacker: &[Person],
    policy: QueryPolicy,
) -> Vec<TargetOutcome> {
    let reference_rows = feature_rows(reference);
    let attacker_rows = feature_rows(attacker);
    let mut outcomes = Vec::with_capacity(attacker.len());

    for (target_id, target) in attacker.iter().enumerate() {
        let mut reference_candidates = initial_cohort(reference, target);
        let mut attacker_candidates = initial_cohort(attacker, target);
        let initial_attacker_candidates = attacker_candidates.clone();
        let initial_attacker_confidence =
            sensitive_confidence_ppm(attacker, &initial_attacker_candidates);
        let mut asked = [false; FEATURE_COUNT];
        let mut accepted_queries = 0;
        let mut stopped_by_guard = false;
        let mut max_income_uplift = 0;
        let mut max_debt_uplift = 0;

        for _ in 0..MAX_ADAPTIVE_QUERIES {
            let Some(feature) = choose_query(&reference_rows, &reference_candidates, &asked)
            else {
                break;
            };

            let Some((yes_reference, no_reference)) = admitted_partition(
                reference,
                &reference_rows,
                &reference_candidates,
                feature,
                policy,
            ) else {
                stopped_by_guard = policy.is_restrictive();
                break;
            };

            asked[feature] = true;
            accepted_queries += 1;
            let answer = attacker_rows[target_id][feature];

            reference_candidates = if answer {
                yes_reference
            } else {
                no_reference
            };
            attacker_candidates.retain(|candidate_id| attacker_rows[*candidate_id][feature] == answer);

            let current_confidence = sensitive_confidence_ppm(attacker, &attacker_candidates);
            max_income_uplift = max_income_uplift.max(
                current_confidence
                    .0
                    .saturating_sub(initial_attacker_confidence.0),
            );
            max_debt_uplift = max_debt_uplift.max(
                current_confidence
                    .1
                    .saturating_sub(initial_attacker_confidence.1),
            );
        }

        outcomes.push(TargetOutcome {
            attacker_candidates,
            accepted_queries,
            stopped_by_guard,
            max_income_confidence_uplift_ppm: max_income_uplift,
            max_debt_confidence_uplift_ppm: max_debt_uplift,
        });
    }

    outcomes
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Summary {
    task_answer_utility_ppm: u64,
    average_candidates_milli: u64,
    p05_candidates: usize,
    p50_candidates: usize,
    p95_candidates: usize,
    below_k_subject_count: usize,
    below_k_subject_rate_ppm: u64,
    empty_candidate_count: usize,
    mean_queries_answered_milli: u64,
    stopped_by_guard_ppm: u64,
    high_income_inference_accuracy_ppm: u64,
    high_income_evaluable_subjects: usize,
    debt_risk_inference_accuracy_ppm: u64,
    debt_risk_evaluable_subjects: usize,
    max_high_income_confidence_uplift_ppm: u64,
    max_debt_risk_confidence_uplift_ppm: u64,
}

fn percentile(sorted: &[usize], percent: usize) -> usize {
    sorted[((sorted.len() - 1) * percent) / 100]
}

fn inference_accuracy_ppm(
    people: &[Person],
    outcomes: &[TargetOutcome],
    label: impl Fn(&Person) -> bool,
) -> (u64, usize) {
    let mut correct = 0;
    let mut evaluable = 0;

    for (target_id, outcome) in outcomes.iter().enumerate() {
        if outcome.attacker_candidates.is_empty() {
            continue;
        }

        let positives = outcome
            .attacker_candidates
            .iter()
            .filter(|candidate_id| label(&people[**candidate_id]))
            .count();
        let predicted_positive = positives * 2 > outcome.attacker_candidates.len();
        correct += usize::from(predicted_positive == label(&people[target_id]));
        evaluable += 1;
    }

    let accuracy = if evaluable == 0 {
        0
    } else {
        (correct as u64 * MILLION) / evaluable as u64
    };
    (accuracy, evaluable)
}

fn summarize(
    people: &[Person],
    outcomes: &[TargetOutcome],
    policy: QueryPolicy,
) -> Summary {
    let mut sizes = outcomes
        .iter()
        .map(|outcome| outcome.attacker_candidates.len())
        .collect::<Vec<_>>();
    sizes.sort_unstable();
    let total_candidates = sizes.iter().sum::<usize>();
    let below_k_subject_count = sizes
        .iter()
        .filter(|size| **size < policy.min_branch_size)
        .count();
    let empty_candidate_count = sizes.iter().filter(|size| **size == 0).count();
    let stopped_count = outcomes
        .iter()
        .filter(|outcome| outcome.stopped_by_guard)
        .count();
    let accepted_query_total = outcomes
        .iter()
        .map(|outcome| outcome.accepted_queries)
        .sum::<usize>();
    let (high_income_accuracy, high_income_evaluable) =
        inference_accuracy_ppm(people, outcomes, Person::high_income);
    let (debt_accuracy, debt_evaluable) =
        inference_accuracy_ppm(people, outcomes, |person| person.debt_risk);

    Summary {
        task_answer_utility_ppm: MILLION,
        average_candidates_milli: (total_candidates as u64 * 1_000) / sizes.len() as u64,
        p05_candidates: percentile(&sizes, 5),
        p50_candidates: percentile(&sizes, 50),
        p95_candidates: percentile(&sizes, 95),
        below_k_subject_count,
        below_k_subject_rate_ppm: (below_k_subject_count as u64 * MILLION) / sizes.len() as u64,
        empty_candidate_count,
        mean_queries_answered_milli: (accepted_query_total as u64 * 1_000) / outcomes.len() as u64,
        stopped_by_guard_ppm: (stopped_count as u64 * MILLION) / outcomes.len() as u64,
        high_income_inference_accuracy_ppm: high_income_accuracy,
        high_income_evaluable_subjects: high_income_evaluable,
        debt_risk_inference_accuracy_ppm: debt_accuracy,
        debt_risk_evaluable_subjects: debt_evaluable,
        max_high_income_confidence_uplift_ppm: outcomes
            .iter()
            .map(|outcome| outcome.max_income_confidence_uplift_ppm)
            .max()
            .unwrap_or(0),
        max_debt_risk_confidence_uplift_ppm: outcomes
            .iter()
            .map(|outcome| outcome.max_debt_confidence_uplift_ppm)
            .max()
            .unwrap_or(0),
    }
}

fn report() -> String {
    let reference = synthetic_population(REFERENCE_SEED);
    let policies = QueryPolicy::ALL;
    let mut output = String::new();
    writeln!(
        &mut output,
        "attacker_population,policy,task_answer_utility_ppm,average_candidates_milli,p05_candidates,p50_candidates,p95_candidates,below_k_subject_count,below_k_subject_rate_ppm,empty_candidate_count,mean_queries_answered_milli,stopped_by_guard_ppm,high_income_inference_accuracy_ppm,high_income_evaluable_subjects,debt_risk_inference_accuracy_ppm,debt_risk_evaluable_subjects,max_high_income_confidence_uplift_ppm,max_debt_risk_confidence_uplift_ppm"
    )
    .expect("write to string");

    for shift in ShiftProfile::ALL {
        let attacker = shift.apply(&reference);
        for policy in policies {
            let outcomes = evaluate(&reference, &attacker, policy);
            let summary = summarize(&attacker, &outcomes, policy);
            write_row(&mut output, shift.name(), policy.name, &summary);
        }
    }

    output
}

fn write_row(output: &mut String, population: &str, policy: &str, metrics: &Summary) {
    writeln!(
        output,
        "{population},{policy},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        metrics.task_answer_utility_ppm,
        metrics.average_candidates_milli,
        metrics.p05_candidates,
        metrics.p50_candidates,
        metrics.p95_candidates,
        metrics.below_k_subject_count,
        metrics.below_k_subject_rate_ppm,
        metrics.empty_candidate_count,
        metrics.mean_queries_answered_milli,
        metrics.stopped_by_guard_ppm,
        metrics.high_income_inference_accuracy_ppm,
        metrics.high_income_evaluable_subjects,
        metrics.debt_risk_inference_accuracy_ppm,
        metrics.debt_risk_evaluable_subjects,
        metrics.max_high_income_confidence_uplift_ppm,
        metrics.max_debt_risk_confidence_uplift_ppm,
    )
    .expect("write to string");
}

fn main() {
    print!("{}", report());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generators_and_shifts_are_deterministic() {
        let reference = synthetic_population(REFERENCE_SEED);
        assert_eq!(reference.len(), POPULATION_SIZE);
        assert_eq!(reference, synthetic_population(REFERENCE_SEED));

        for shift in ShiftProfile::ALL {
            assert_eq!(shift.apply(&reference), shift.apply(&reference));
        }
    }

    #[test]
    fn matched_reference_control_respects_the_branch_guard() {
        let reference = synthetic_population(REFERENCE_SEED);
        let attacker = ShiftProfile::MatchedControl.apply(&reference);
        let policy = QueryPolicy {
            name: "branch_k32",
            min_branch_size: 32,
            max_sensitive_uplift_ppm: MILLION,
        };
        let outcomes = evaluate(&reference, &attacker, policy);
        let summary = summarize(&attacker, &outcomes, policy);

        assert_eq!(summary.task_answer_utility_ppm, MILLION);
        assert_eq!(summary.below_k_subject_count, 0);
        assert_eq!(summary.empty_candidate_count, 0);
    }

    #[test]
    fn independent_population_shift_exposes_k_guard_miscalibration() {
        let reference = synthetic_population(REFERENCE_SEED);
        let attacker = ShiftProfile::IndependentSeed.apply(&reference);
        let policy = QueryPolicy {
            name: "branch_k32",
            min_branch_size: 32,
            max_sensitive_uplift_ppm: MILLION,
        };
        let outcomes = evaluate(&reference, &attacker, policy);
        let summary = summarize(&attacker, &outcomes, policy);

        assert_eq!(summary.task_answer_utility_ppm, MILLION);
        assert_eq!(summary.below_k_subject_count, 83);
        assert_eq!(summary.below_k_subject_rate_ppm, 162_109);
    }

    #[test]
    fn correlation_reversal_exposes_k_guard_miscalibration_but_dual_guard_holds_this_fixture() {
        let reference = synthetic_population(REFERENCE_SEED);
        let attacker = ShiftProfile::DebtCorrelationReversed.apply(&reference);
        let branch_policy = QueryPolicy {
            name: "branch_k32",
            min_branch_size: 32,
            max_sensitive_uplift_ppm: MILLION,
        };
        let branch_summary =
            summarize(&attacker, &evaluate(&reference, &attacker, branch_policy), branch_policy);
        assert_eq!(branch_summary.below_k_subject_count, 59);
        assert_eq!(branch_summary.below_k_subject_rate_ppm, 115_234);

        let dual_policy = QueryPolicy {
            name: "dual_k32_uplift100k",
            min_branch_size: 32,
            max_sensitive_uplift_ppm: 100_000,
        };
        let dual_summary =
            summarize(&attacker, &evaluate(&reference, &attacker, dual_policy), dual_policy);
        assert_eq!(dual_summary.below_k_subject_count, 0);
        assert_eq!(dual_summary.task_answer_utility_ppm, MILLION);
    }

    #[test]
    fn all_shift_profiles_preserve_task_answer_utility() {
        let reference = synthetic_population(REFERENCE_SEED);
        for shift in ShiftProfile::ALL {
            let attacker = shift.apply(&reference);
            for policy in QueryPolicy::ALL {
                let outcomes = evaluate(&reference, &attacker, policy);
                let summary = summarize(&attacker, &outcomes, policy);
                assert_eq!(
                    summary.task_answer_utility_ppm, MILLION,
                    "utility changed for {} / {}",
                    shift.name(),
                    policy.name
                );
            }
        }
    }

    #[test]
    fn report_is_byte_for_byte_deterministic() {
        assert_eq!(report(), report());
    }
}
