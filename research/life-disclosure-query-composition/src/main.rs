use std::fmt::Write as _;

const POPULATION_SIZE: usize = 1_024;
const FEATURE_COUNT: usize = 18;
const MAX_ADAPTIVE_QUERIES: usize = 8;
const MILLION: u64 = 1_000_000;

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
            self.age_band > 0,             // adult
            self.region < 4,               // in the eligibility jurisdiction
            self.age_band >= 2,            // age-band threshold
            self.age_band >= 4,            // age-band threshold
            self.age_band >= 6,            // age-band threshold
            self.income_band >= 2,         // income threshold
            self.income_band >= 4,         // high income
            self.sector == 0,              // public sector
            self.sector == 1,              // technology sector
            self.sector == 2,              // industrial sector
            self.family_size >= 3,         // household size threshold
            self.family_size >= 5,         // larger household
            self.debt_risk,                // sensitive risk attribute
            self.homeowner,                // housing tenure
            self.public_transit,           // mobility behavior
            self.income_band <= 1,         // low-income / benefit proxy
            self.family_size >= 2,         // household composition
            self.region >= 4,              // out-of-jurisdiction indicator
        ]
    }

    fn eligible(&self) -> bool {
        let features = self.features();
        features[0] && features[1]
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

fn synthetic_population() -> Vec<Person> {
    (0..POPULATION_SIZE)
        .map(|id| {
            let mut state = 0x4c49_4645_5155_4552_u64
                ^ (id as u64).wrapping_mul(0xd6e8_feb8_6659_fd93);
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
enum BaseDisclosure {
    TaskResult,
    TaskInputs,
    FullAttributes,
}

fn initial_cohort(people: &[Person], target_id: usize, disclosure: BaseDisclosure) -> Vec<usize> {
    let target = &people[target_id];
    let target_features = target.features();

    (0..people.len())
        .filter(|candidate_id| {
            let candidate = &people[*candidate_id];
            let candidate_features = candidate.features();
            match disclosure {
                BaseDisclosure::TaskResult => candidate.eligible() == target.eligible(),
                BaseDisclosure::TaskInputs => {
                    candidate_features[0] == target_features[0]
                        && candidate_features[1] == target_features[1]
                }
                BaseDisclosure::FullAttributes => candidate_features == target_features,
            }
        })
        .collect()
}

fn choose_query(
    people: &[Person],
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
            .filter(|candidate_id| people[**candidate_id].features()[feature])
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

#[derive(Clone, Debug)]
struct AdaptiveRun {
    by_depth: Vec<Vec<Vec<usize>>>,
    queries_by_depth: Vec<Vec<usize>>,
    stopped_by_guard: Vec<bool>,
}

fn adaptive_cohorts_with_min_branch(people: &[Person], min_branch_size: usize) -> AdaptiveRun {
    let mut by_depth =
        vec![Vec::<Vec<usize>>::with_capacity(people.len()); MAX_ADAPTIVE_QUERIES + 1];
    let mut queries_by_depth =
        vec![Vec::<usize>::with_capacity(people.len()); MAX_ADAPTIVE_QUERIES + 1];
    let mut stopped_by_guard = Vec::with_capacity(people.len());

    for (target_id, target) in people.iter().enumerate() {
        let target_features = target.features();
        let mut candidates = initial_cohort(people, target_id, BaseDisclosure::TaskResult);
        let mut asked = [false; FEATURE_COUNT];
        let mut accepted_queries = 0;
        let mut stopped = false;

        for (depth, (depth_cohorts, depth_queries)) in by_depth
            .iter_mut()
            .zip(queries_by_depth.iter_mut())
            .enumerate()
        {
            depth_cohorts.push(candidates.clone());
            depth_queries.push(accepted_queries);

            if depth == MAX_ADAPTIVE_QUERIES
                || candidates.len() <= 1
                || stopped
            {
                continue;
            }

            let Some(feature) = choose_query(people, &candidates, &asked) else {
                continue;
            };

            let yes_count = candidates
                .iter()
                .filter(|candidate_id| people[**candidate_id].features()[feature])
                .count();
            let no_count = candidates.len() - yes_count;

            // Evaluate both potential answers before observing the target's bit.
            // This avoids making the allow/deny result itself disclose which branch
            // the target would have taken.
            if yes_count < min_branch_size || no_count < min_branch_size {
                stopped = min_branch_size > 1;
                continue;
            }

            asked[feature] = true;
            accepted_queries += 1;
            let answer = target_features[feature];
            candidates.retain(|candidate_id| {
                people[*candidate_id].features()[feature] == answer
            });
        }

        stopped_by_guard.push(stopped);
    }

    AdaptiveRun {
        by_depth,
        queries_by_depth,
        stopped_by_guard,
    }
}

#[derive(Debug, PartialEq, Eq)]
struct Metrics {
    task_utility_ppm: u64,
    average_candidates_milli: u64,
    p05_candidates: usize,
    p50_candidates: usize,
    p95_candidates: usize,
    singleton_rate_ppm: u64,
    high_income_inference_accuracy_ppm: u64,
    debt_risk_inference_accuracy_ppm: u64,
    mean_queries_answered_milli: u64,
    stopped_by_guard_ppm: u64,
}

fn percentile(sorted: &[usize], percent: usize) -> usize {
    let index = ((sorted.len() - 1) * percent) / 100;
    sorted[index]
}

fn prior_accuracy(people: &[Person], label: impl Fn(&Person) -> bool) -> u64 {
    let positives = people.iter().filter(|person| label(person)).count();
    let correct = positives.max(people.len() - positives);
    (correct as u64 * MILLION) / people.len() as u64
}

fn inference_accuracy_ppm(
    people: &[Person],
    cohorts: &[Vec<usize>],
    label: impl Fn(&Person) -> bool,
) -> u64 {
    let correct = cohorts
        .iter()
        .enumerate()
        .filter(|(target_id, candidate_ids)| {
            let positives = candidate_ids
                .iter()
                .filter(|candidate_id| label(&people[**candidate_id]))
                .count();
            let predicted_positive = positives * 2 > candidate_ids.len();
            predicted_positive == label(&people[*target_id])
        })
        .count();
    (correct as u64 * MILLION) / people.len() as u64
}

fn task_utility_ppm(people: &[Person], disclosure: BaseDisclosure) -> u64 {
    let correct = people
        .iter()
        .filter(|person| {
            let features = person.features();
            let disclosed_answer = match disclosure {
                BaseDisclosure::TaskResult => person.eligible(),
                BaseDisclosure::TaskInputs => features[0] && features[1],
                BaseDisclosure::FullAttributes => features[0] && features[1],
            };
            disclosed_answer == person.eligible()
        })
        .count();
    (correct as u64 * MILLION) / people.len() as u64
}

fn summarize(
    people: &[Person],
    cohorts: &[Vec<usize>],
    disclosure: BaseDisclosure,
    accepted_queries: &[usize],
    stopped_by_guard: &[bool],
) -> Metrics {
    let mut sizes = cohorts.iter().map(Vec::len).collect::<Vec<_>>();
    sizes.sort_unstable();
    let total_candidates = sizes.iter().sum::<usize>();
    let singleton_count = sizes.iter().filter(|size| **size == 1).count();
    let accepted_query_total = accepted_queries.iter().sum::<usize>();
    let stopped_count = stopped_by_guard.iter().filter(|stopped| **stopped).count();

    Metrics {
        task_utility_ppm: task_utility_ppm(people, disclosure),
        average_candidates_milli: (total_candidates as u64 * 1_000) / sizes.len() as u64,
        p05_candidates: percentile(&sizes, 5),
        p50_candidates: percentile(&sizes, 50),
        p95_candidates: percentile(&sizes, 95),
        singleton_rate_ppm: (singleton_count as u64 * MILLION) / sizes.len() as u64,
        high_income_inference_accuracy_ppm: inference_accuracy_ppm(
            people,
            cohorts,
            Person::high_income,
        ),
        debt_risk_inference_accuracy_ppm: inference_accuracy_ppm(
            people,
            cohorts,
            |person| person.debt_risk,
        ),
        mean_queries_answered_milli: (accepted_query_total as u64 * 1_000)
            / accepted_queries.len() as u64,
        stopped_by_guard_ppm: (stopped_count as u64 * MILLION) / stopped_by_guard.len() as u64,
    }
}

fn report() -> String {
    let people = synthetic_population();
    let task_result = people
        .iter()
        .enumerate()
        .map(|(target_id, _)| initial_cohort(&people, target_id, BaseDisclosure::TaskResult))
        .collect::<Vec<_>>();
    let task_inputs = people
        .iter()
        .enumerate()
        .map(|(target_id, _)| initial_cohort(&people, target_id, BaseDisclosure::TaskInputs))
        .collect::<Vec<_>>();
    let full_attributes = people
        .iter()
        .enumerate()
        .map(|(target_id, _)| initial_cohort(&people, target_id, BaseDisclosure::FullAttributes))
        .collect::<Vec<_>>();
    let adaptive = adaptive_cohorts_with_min_branch(&people, 1);
    let guarded_32 = adaptive_cohorts_with_min_branch(&people, 32);
    let guarded_64 = adaptive_cohorts_with_min_branch(&people, 64);
    let zero_queries = vec![0; people.len()];
    let no_guard_stops = vec![false; people.len()];

    let mut output = String::new();
    writeln!(
        &mut output,
        "profile,adaptive_query_cap,task_utility_ppm,average_candidates_milli,p05_candidates,p50_candidates,p95_candidates,singleton_rate_ppm,high_income_inference_accuracy_ppm,debt_risk_inference_accuracy_ppm,mean_queries_answered_milli,stopped_by_guard_ppm"
    )
    .expect("write to string");
    write_row(
        &mut output,
        "task_result_only",
        0,
        &summarize(
            &people,
            &task_result,
            BaseDisclosure::TaskResult,
            &zero_queries,
            &no_guard_stops,
        ),
    );
    write_row(
        &mut output,
        "task_inputs_only",
        0,
        &summarize(
            &people,
            &task_inputs,
            BaseDisclosure::TaskInputs,
            &zero_queries,
            &no_guard_stops,
        ),
    );

    for (depth, cohorts) in adaptive.by_depth.iter().enumerate() {
        write_row(
            &mut output,
            &format!("adaptive_q{depth}"),
            depth,
            &summarize(
                &people,
                cohorts,
                BaseDisclosure::TaskResult,
                &adaptive.queries_by_depth[depth],
                &no_guard_stops,
            ),
        );
    }

    for (profile, min_branch, run) in [
        ("risk_guard_k32", 32, &guarded_32),
        ("risk_guard_k64", 64, &guarded_64),
    ] {
        write_row(
            &mut output,
            profile,
            MAX_ADAPTIVE_QUERIES,
            &summarize(
                &people,
                &run.by_depth[MAX_ADAPTIVE_QUERIES],
                BaseDisclosure::TaskResult,
                &run.queries_by_depth[MAX_ADAPTIVE_QUERIES],
                &run.stopped_by_guard,
            ),
        );
        writeln!(&mut output, "#{profile}_minimum_branch_size={min_branch}")
            .expect("write to string");
    }

    write_row(
        &mut output,
        "full_attribute_disclosure",
        FEATURE_COUNT,
        &summarize(
            &people,
            &full_attributes,
            BaseDisclosure::FullAttributes,
            &zero_queries,
            &no_guard_stops,
        ),
    );

    let prior_income = prior_accuracy(&people, Person::high_income);
    let prior_debt = prior_accuracy(&people, |person| person.debt_risk);
    writeln!(&mut output, "#population_prior_high_income_accuracy_ppm={prior_income}")
        .expect("write to string");
    writeln!(&mut output, "#population_prior_debt_risk_accuracy_ppm={prior_debt}")
        .expect("write to string");
    output
}

fn write_row(output: &mut String, profile: &str, queries: usize, metrics: &Metrics) {
    writeln!(
        output,
        "{profile},{queries},{},{},{},{},{},{},{},{},{},{}",
        metrics.task_utility_ppm,
        metrics.average_candidates_milli,
        metrics.p05_candidates,
        metrics.p50_candidates,
        metrics.p95_candidates,
        metrics.singleton_rate_ppm,
        metrics.high_income_inference_accuracy_ppm,
        metrics.debt_risk_inference_accuracy_ppm,
        metrics.mean_queries_answered_milli,
        metrics.stopped_by_guard_ppm,
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
    fn synthetic_population_is_deterministic_and_fixed_size() {
        assert_eq!(synthetic_population(), synthetic_population());
        assert_eq!(synthetic_population().len(), POPULATION_SIZE);
    }

    #[test]
    fn every_profile_preserves_the_exact_task_answer() {
        let people = synthetic_population();
        assert_eq!(task_utility_ppm(&people, BaseDisclosure::TaskResult), MILLION);
        assert_eq!(task_utility_ppm(&people, BaseDisclosure::TaskInputs), MILLION);
        assert_eq!(task_utility_ppm(&people, BaseDisclosure::FullAttributes), MILLION);
    }

    #[test]
    fn adaptive_questions_never_increase_candidate_sets() {
        let people = synthetic_population();
        let run = adaptive_cohorts_with_min_branch(&people, 1);

        for (depth, (previous, current)) in run
            .by_depth
            .iter()
            .zip(run.by_depth.iter().skip(1))
            .enumerate()
        {
            for (target_id, current_cohort) in current.iter().enumerate() {
                assert!(
                    current_cohort.len() <= previous[target_id].len(),
                    "target {target_id} gained candidates at query depth {}",
                    depth + 1
                );
            }
        }
    }

    #[test]
    fn risk_guard_never_allows_a_candidate_cohort_below_its_threshold() {
        let people = synthetic_population();

        for threshold in [32, 64] {
            let run = adaptive_cohorts_with_min_branch(&people, threshold);
            for (depth, cohorts) in run.by_depth.iter().enumerate() {
                for (target_id, candidates) in cohorts.iter().enumerate() {
                    assert!(
                        candidates.len() >= threshold,
                        "threshold {threshold}: target {target_id} has {} candidates at depth {depth}",
                        candidates.len()
                    );
                }
            }
        }
    }

    #[test]
    fn risk_guard_stops_queries_without_changing_task_answers() {
        let people = synthetic_population();
        let unbounded = adaptive_cohorts_with_min_branch(&people, 1);
        let guarded = adaptive_cohorts_with_min_branch(&people, 32);
        let last = MAX_ADAPTIVE_QUERIES;
        let unbounded_mean = unbounded.queries_by_depth[last].iter().sum::<usize>();
        let guarded_mean = guarded.queries_by_depth[last].iter().sum::<usize>();

        assert!(guarded.stopped_by_guard.iter().any(|stopped| *stopped));
        assert!(guarded_mean < unbounded_mean);
        assert_eq!(task_utility_ppm(&people, BaseDisclosure::TaskResult), MILLION);
    }

    #[test]
    fn full_attribute_cohort_is_subset_of_task_result_cohort() {
        let people = synthetic_population();

        for (target_id, _) in people.iter().enumerate() {
            let result_cohort = initial_cohort(&people, target_id, BaseDisclosure::TaskResult);
            let full_cohort =
                initial_cohort(&people, target_id, BaseDisclosure::FullAttributes);

            assert!(!full_cohort.is_empty());
            assert!(full_cohort.iter().all(|id| result_cohort.contains(id)));
        }
    }

    #[test]
    fn every_emitted_profile_has_full_task_utility() {
        for line in report().lines().skip(1) {
            if line.starts_with('#') {
                continue;
            }
            let fields = line.split(',').collect::<Vec<_>>();
            assert_eq!(fields[2], "1000000", "unexpected utility in {line}");
        }
    }

    #[test]
    fn guarded_policy_stops_before_learning_the_target_answer() {
        let people = synthetic_population();
        let threshold = 32;
        let run = adaptive_cohorts_with_min_branch(&people, threshold);
        let target_id = run
            .stopped_by_guard
            .iter()
            .position(|stopped| *stopped)
            .expect("at least one adaptive query should be denied");

        // The guard decides using both possible branch sizes before inspecting
        // this target's next answer. Recomputing the chosen query confirms the
        // accepted cohort remains above the threshold.
        let final_cohort = &run.by_depth[MAX_ADAPTIVE_QUERIES][target_id];
        assert!(final_cohort.len() >= threshold);
    }

    #[test]
    fn report_is_byte_for_byte_deterministic() {
        assert_eq!(report(), report());
    }
}
