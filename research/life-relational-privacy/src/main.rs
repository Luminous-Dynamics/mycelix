use std::fmt::Write as _;

const BASE_SEED: u64 = 0x5245_4c41_5449_4f4e;
const TRAIN_COHORTS: usize = 96;
const TEST_COHORTS: usize = 96;
const COARSENING_FACTOR: usize = 4;
const PPM: u64 = 1_000_000;
const CONFIDENCE_THRESHOLDS: [u64; 5] = [500_000, 625_000, 750_000, 875_000, 950_000];
const MANIFEST: &str = include_str!("../manifest.json");

#[derive(Clone, Debug, PartialEq, Eq)]
struct Subject {
    cohort_id: u64,
    coarse_bucket: u64,
    secondary_group_id: u64,
    protected: bool,
    independent_secret: bool,
    peer_proxy: bool,
    randomized_peer_proxy: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct AttackerView {
    cohort_id: u64,
    coarse_bucket: u64,
    secondary_group_id: u64,
    peer_proxy: bool,
    randomized_peer_proxy: bool,
}

impl From<&Subject> for AttackerView {
    fn from(subject: &Subject) -> Self {
        Self {
            cohort_id: subject.cohort_id,
            coarse_bucket: subject.coarse_bucket,
            secondary_group_id: subject.secondary_group_id,
            peer_proxy: subject.peer_proxy,
            randomized_peer_proxy: subject.randomized_peer_proxy,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PredictionRecord {
    predicted: bool,
    confidence_score_ppm: u64,
    peer_observations_used: u64,
    peer_observation_delta_vs_exact_cohort: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Profile {
    PopulationPrior,
    ExactCohortPeerMajority,
    CoarseNeighborhoodPeerMajority,
    OverlappingGroupUnionPeerMajority,
    RandomizedPeerNegativeControl,
    UncorrelatedTargetNegativeControl,
}

impl Profile {
    const ALL: [Self; 5] = [
        Self::PopulationPrior,
        Self::ExactCohortPeerMajority,
        Self::CoarseNeighborhoodPeerMajority,
        Self::OverlappingGroupUnionPeerMajority,
        Self::RandomizedPeerNegativeControl,
        Self::UncorrelatedTargetNegativeControl,
    ];

    const fn name(self) -> &'static str {
        match self {
            Self::PopulationPrior => "population_prior",
            Self::ExactCohortPeerMajority => "exact_cohort_peer_majority",
            Self::CoarseNeighborhoodPeerMajority => "coarse_neighborhood_peer_majority",
            Self::OverlappingGroupUnionPeerMajority => "overlapping_group_union_peer_majority",
            Self::RandomizedPeerNegativeControl => "randomized_peer_negative_control",
            Self::UncorrelatedTargetNegativeControl => "uncorrelated_target_negative_control",
        }
    }

    const fn uses_independent_target(self) -> bool {
        matches!(self, Self::UncorrelatedTargetNegativeControl)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct OutcomeRecord {
    truth: bool,
    predicted: bool,
    prior_predicted: bool,
    confidence_score_ppm: u64,
    peer_observations_used: u64,
    peer_observation_delta_vs_exact_cohort: i64,
    direct_target_protected_disclosures: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Summary {
    target_count: u64,
    positive_targets: u64,
    correct: u64,
    prior_correct: u64,
    true_positive: u64,
    true_negative: u64,
    false_positive: u64,
    false_negative: u64,
    peer_observations_used: u64,
    peer_observation_delta_vs_exact_cohort: i64,
    direct_target_protected_disclosures: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Audit {
    target_count: u64,
    correct_count: u64,
    direct_target_protected_disclosures: u64,
}

#[derive(Clone, Copy, Debug)]
struct Rng {
    state: u64,
}

impl Rng {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 {
                0xa076_1d64_78bd_642f
            } else {
                seed
            },
        }
    }

    fn next_u64(&mut self) -> u64 {
        // xorshift64*: fixed algorithm, no platform RNG dependency.
        self.state ^= self.state >> 12;
        self.state ^= self.state << 25;
        self.state ^= self.state >> 27;
        self.state.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }

    fn bit(&mut self) -> bool {
        self.next_u64() & 1 == 1
    }

    fn chance_ppm(&mut self, probability_ppm: u32) -> bool {
        u64::from(probability_ppm) > self.next_u64() % PPM
    }
}

fn seeded(seed_part: u64, cohort_size: usize, signal_ppm: u32, split: u64) -> u64 {
    BASE_SEED
        ^ seed_part
        ^ (cohort_size as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)
        ^ u64::from(signal_ppm).rotate_left(17)
        ^ split
}

fn generate_population(
    group_count: usize,
    cohort_size: usize,
    peer_signal_ppm: u32,
    cohort_id_base: u64,
    seed: u64,
) -> Vec<Subject> {
    let mut rng = Rng::new(seed);
    let mut subjects = Vec::with_capacity(group_count.saturating_mul(cohort_size));

    let secondary_group_count = group_count.saturating_div(4).max(1);
    for group in 0..group_count {
        let latent = rng.bit();
        let cohort_id = cohort_id_base + group as u64;
        let coarse_bucket =
            cohort_id_base / COARSENING_FACTOR as u64 + (group / COARSENING_FACTOR) as u64;

        for member in 0..cohort_size {
            // A second membership crosses the primary cohort partition. It is
            // independent of the protected signal generation and acts as
            // explicit relation-metadata side information to the attacker.
            let secondary_group_id = ((group * 7 + member * 11) % secondary_group_count) as u64;
            let protected = if rng.chance_ppm(peer_signal_ppm) {
                latent
            } else {
                !latent
            };
            let peer_proxy = if rng.chance_ppm(peer_signal_ppm) {
                latent
            } else {
                !latent
            };

            subjects.push(Subject {
                cohort_id,
                coarse_bucket,
                secondary_group_id,
                protected,
                independent_secret: rng.bit(),
                peer_proxy,
                randomized_peer_proxy: rng.bit(),
            });
        }
    }

    subjects
}

fn truth_for(subject: &Subject, profile: Profile) -> bool {
    if profile.uses_independent_target() {
        subject.independent_secret
    } else {
        subject.protected
    }
}

fn prior_for(training: &[Subject], profile: Profile) -> bool {
    let positive = training
        .iter()
        .filter(|subject| truth_for(subject, profile))
        .count();
    positive.saturating_mul(2) >= training.len()
}

fn prior_confidence_ppm(training: &[Subject], profile: Profile) -> u64 {
    if training.is_empty() {
        return 0;
    }
    let positive = training
        .iter()
        .filter(|subject| truth_for(subject, profile))
        .count() as u64;
    let total = training.len() as u64;
    ratio_ppm(positive.max(total - positive), total)
}

fn majority(values: impl Iterator<Item = bool>, prior: bool) -> bool {
    let mut positive = 0_u64;
    let mut negative = 0_u64;
    for value in values {
        if value {
            positive += 1;
        } else {
            negative += 1;
        }
    }

    if positive == negative {
        prior
    } else {
        positive > negative
    }
}

fn peer_signal(subject: &AttackerView, profile: Profile) -> bool {
    if matches!(profile, Profile::RandomizedPeerNegativeControl) {
        subject.randomized_peer_proxy
    } else {
        subject.peer_proxy
    }
}

// This function receives only the intentionally visible fields. It cannot
// inspect a target's protected label or independent evaluation-only secret.
fn predict_attacker(
    views: &[AttackerView],
    cohort_size: usize,
    profile: Profile,
    prior_prediction: bool,
    prior_confidence: u64,
) -> Vec<PredictionRecord> {
    let mut predictions = Vec::with_capacity(views.len());

    for (target_index, target) in views.iter().enumerate() {
        let peer_indices = if matches!(profile, Profile::PopulationPrior) {
            Vec::new()
        } else {
            views
                .iter()
                .enumerate()
                .filter_map(|(candidate_index, candidate)| {
                    if candidate_index == target_index {
                        return None;
                    }

                    let related = match profile {
                        Profile::CoarseNeighborhoodPeerMajority => {
                            candidate.coarse_bucket == target.coarse_bucket
                        }
                        Profile::OverlappingGroupUnionPeerMajority => {
                            candidate.cohort_id == target.cohort_id
                                || candidate.secondary_group_id == target.secondary_group_id
                        }
                        _ => candidate.cohort_id == target.cohort_id,
                    };

                    if related { Some(candidate_index) } else { None }
                })
                .collect::<Vec<_>>()
        };

        let peer_signals = peer_indices
            .iter()
            .map(|index| peer_signal(&views[*index], profile))
            .collect::<Vec<_>>();
        let predicted = if matches!(profile, Profile::PopulationPrior) {
            prior_prediction
        } else {
            majority(peer_signals.iter().copied(), prior_prediction)
        };
        let confidence_score_ppm = if peer_signals.is_empty() {
            prior_confidence
        } else {
            let positive_votes = peer_signals.iter().filter(|value| **value).count() as u64;
            let negative_votes = peer_signals.len() as u64 - positive_votes;
            ratio_ppm(
                positive_votes.max(negative_votes),
                peer_signals.len() as u64,
            )
        };

        let peer_count = peer_indices.len() as u64;
        let exact_peer_count = cohort_size.saturating_sub(1) as u64;
        predictions.push(PredictionRecord {
            predicted,
            confidence_score_ppm,
            peer_observations_used: peer_count,
            peer_observation_delta_vs_exact_cohort: peer_count as i64 - exact_peer_count as i64,
        });
    }

    predictions
}

fn evaluate_profile(
    training: &[Subject],
    test: &[Subject],
    cohort_size: usize,
    profile: Profile,
) -> Vec<OutcomeRecord> {
    let prior_prediction = prior_for(training, profile);
    let prior_confidence = prior_confidence_ppm(training, profile);

    // Drop protected labels before constructing the attacker's data plane.
    let views = test.iter().map(AttackerView::from).collect::<Vec<_>>();
    let predictions = predict_attacker(
        &views,
        cohort_size,
        profile,
        prior_prediction,
        prior_confidence,
    );

    // Evaluation truth is joined only after predictions have been produced.
    test.iter()
        .zip(predictions)
        .map(|(target, prediction)| OutcomeRecord {
            truth: truth_for(target, profile),
            predicted: prediction.predicted,
            prior_predicted: prior_prediction,
            confidence_score_ppm: prediction.confidence_score_ppm,
            peer_observations_used: prediction.peer_observations_used,
            peer_observation_delta_vs_exact_cohort: prediction
                .peer_observation_delta_vs_exact_cohort,
            // No direct protected-value/claim field is passed to the attacker.
            direct_target_protected_disclosures: 0,
        })
        .collect()
}

fn summarize(outcomes: &[OutcomeRecord]) -> Summary {
    let mut result = Summary::default();

    for record in outcomes {
        result.target_count += 1;
        result.positive_targets += record.truth as u64;
        result.correct += (record.predicted == record.truth) as u64;
        result.prior_correct += (record.prior_predicted == record.truth) as u64;
        result.peer_observations_used += record.peer_observations_used;
        result.peer_observation_delta_vs_exact_cohort +=
            record.peer_observation_delta_vs_exact_cohort;
        result.direct_target_protected_disclosures += record.direct_target_protected_disclosures;

        match (record.truth, record.predicted) {
            (true, true) => result.true_positive += 1,
            (false, false) => result.true_negative += 1,
            (false, true) => result.false_positive += 1,
            (true, false) => result.false_negative += 1,
        }
    }

    result
}

// Independent post-hoc audit reconstructed from per-target outcome records.
// It does not read Summary or any evaluator-side counters.
fn audit_outcomes(outcomes: &[OutcomeRecord]) -> Audit {
    let mut audit = Audit::default();
    for outcome in outcomes {
        audit.target_count += 1;
        if outcome.predicted == outcome.truth {
            audit.correct_count += 1;
        }
        audit.direct_target_protected_disclosures += outcome.direct_target_protected_disclosures;
    }
    audit
}

fn ratio_ppm(numerator: u64, denominator: u64) -> u64 {
    if denominator == 0 {
        0
    } else {
        numerator.saturating_mul(PPM) / denominator
    }
}

fn accuracy_ppm(summary: &Summary) -> u64 {
    ratio_ppm(summary.correct, summary.target_count)
}

fn prior_accuracy_ppm(summary: &Summary) -> u64 {
    ratio_ppm(summary.prior_correct, summary.target_count)
}

fn balanced_accuracy_ppm(summary: &Summary) -> u64 {
    let positives = summary.true_positive + summary.false_negative;
    let negatives = summary.true_negative + summary.false_positive;
    if positives == 0 || negatives == 0 {
        return 0;
    }

    let sensitivity = ratio_ppm(summary.true_positive, positives);
    let specificity = ratio_ppm(summary.true_negative, negatives);
    (sensitivity + specificity) / 2
}

fn selective_metrics(outcomes: &[OutcomeRecord], threshold_ppm: u64) -> (u64, u64, u64) {
    let mut accepted = 0_u64;
    let mut correct = 0_u64;
    for outcome in outcomes {
        if outcome.confidence_score_ppm >= threshold_ppm {
            accepted += 1;
            correct += (outcome.predicted == outcome.truth) as u64;
        }
    }

    let total = outcomes.len() as u64;
    let coverage_ppm = ratio_ppm(accepted, total);
    let selective_accuracy_ppm = ratio_ppm(correct, accepted);
    (
        coverage_ppm,
        selective_accuracy_ppm,
        total.saturating_sub(accepted),
    )
}

fn render_report() -> String {
    debug_assert!(!MANIFEST.is_empty());
    let mut report = String::new();
    writeln!(
        &mut report,
        "profile,cohort_size,peer_signal_ppm,coarsening_factor,target_count,positive_targets,accuracy_ppm,balanced_accuracy_ppm,prior_accuracy_ppm,lift_vs_prior_ppm,mean_peer_observations,peer_observation_delta_vs_exact_cohort,direct_target_protected_disclosures,audit_target_count,audit_correct_count,audit_direct_target_protected_disclosures,decision_threshold_ppm,selective_coverage_ppm,selective_accuracy_ppm,abstained_targets"
    )
    .expect("writing to String cannot fail");

    for cohort_size in [4_usize, 8, 16, 32] {
        for signal_ppm in [550_000_u32, 650_000, 750_000, 850_000, 950_000] {
            let training = generate_population(
                TRAIN_COHORTS,
                cohort_size,
                signal_ppm,
                0,
                seeded(0, cohort_size, signal_ppm, 0x1111),
            );
            // Training and test cohorts have disjoint, explicit identifier ranges.
            let test = generate_population(
                TEST_COHORTS,
                cohort_size,
                signal_ppm,
                1_000_000,
                seeded(0, cohort_size, signal_ppm, 0x9999),
            );

            for profile in Profile::ALL {
                let outcomes = evaluate_profile(&training, &test, cohort_size, profile);
                let summary = summarize(&outcomes);
                let audit = audit_outcomes(&outcomes);
                let accuracy = accuracy_ppm(&summary);
                let prior_accuracy = prior_accuracy_ppm(&summary);
                let lift = accuracy as i64 - prior_accuracy as i64;
                let mean_peers = summary.peer_observations_used / summary.target_count.max(1);

                for threshold_ppm in CONFIDENCE_THRESHOLDS {
                    let (coverage_ppm, selective_accuracy, abstained) =
                        selective_metrics(&outcomes, threshold_ppm);
                    writeln!(
                        &mut report,
                        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                        profile.name(),
                        cohort_size,
                        signal_ppm,
                        COARSENING_FACTOR,
                        summary.target_count,
                        summary.positive_targets,
                        accuracy,
                        balanced_accuracy_ppm(&summary),
                        prior_accuracy,
                        lift,
                        mean_peers,
                        summary.peer_observation_delta_vs_exact_cohort,
                        summary.direct_target_protected_disclosures,
                        audit.target_count,
                        audit.correct_count,
                        audit.direct_target_protected_disclosures,
                        threshold_ppm,
                        coverage_ppm,
                        selective_accuracy,
                        abstained,
                    )
                    .expect("writing to String cannot fail");
                }
            }
        }
    }

    report
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    fn populations(
        cohort_size: usize,
        signal_ppm: u32,
        group_count: usize,
        seed_part: u64,
    ) -> (Vec<Subject>, Vec<Subject>) {
        let training = generate_population(
            group_count,
            cohort_size,
            signal_ppm,
            0,
            seeded(seed_part, cohort_size, signal_ppm, 0x1111),
        );
        let test = generate_population(
            group_count,
            cohort_size,
            signal_ppm,
            1_000_000,
            seeded(seed_part, cohort_size, signal_ppm, 0x9999),
        );
        (training, test)
    }

    #[test]
    fn manifest_freezes_attacker_arms_and_selective_thresholds() {
        assert!(MANIFEST.contains("\"target_protected_value_available_to_attacker\": false"));
        for profile in Profile::ALL {
            assert!(
                MANIFEST.contains(profile.name()),
                "missing {}",
                profile.name()
            );
        }
        for threshold_ppm in CONFIDENCE_THRESHOLDS {
            let threshold = threshold_ppm.to_string();
            assert!(MANIFEST.contains(threshold.as_str()), "missing {threshold}");
        }
    }

    #[test]
    fn identical_seed_and_manifest_produce_identical_report() {
        assert!(!MANIFEST.is_empty());
        assert_eq!(render_report(), render_report());
    }

    #[test]
    fn training_and_test_cohort_ids_are_disjoint() {
        let (training, test) = populations(8, 850_000, 64, 7);
        let training_ids: BTreeSet<u64> =
            training.iter().map(|subject| subject.cohort_id).collect();
        let test_ids: BTreeSet<u64> = test.iter().map(|subject| subject.cohort_id).collect();
        assert!(training_ids.is_disjoint(&test_ids));
        assert!(test_ids.iter().all(|id| *id >= 1_000_000));
    }

    #[test]
    fn changing_protected_labels_cannot_change_attacker_predictions() {
        let (training, test) = populations(8, 850_000, 64, 11);
        let mut label_flipped = test.clone();
        for subject in &mut label_flipped {
            subject.protected = !subject.protected;
            subject.independent_secret = !subject.independent_secret;
        }

        let original_views = test.iter().map(AttackerView::from).collect::<Vec<_>>();
        let flipped_views = label_flipped
            .iter()
            .map(AttackerView::from)
            .collect::<Vec<_>>();
        assert_eq!(original_views, flipped_views);

        for profile in Profile::ALL {
            let prior_prediction = prior_for(&training, profile);
            let prior_confidence = prior_confidence_ppm(&training, profile);
            let original = predict_attacker(
                &original_views,
                8,
                profile,
                prior_prediction,
                prior_confidence,
            );
            let flipped = predict_attacker(
                &flipped_views,
                8,
                profile,
                prior_prediction,
                prior_confidence,
            );
            assert_eq!(original, flipped, "profile {}", profile.name());
        }
    }

    #[test]
    fn target_protected_claim_disclosure_is_zero_for_every_profile() {
        let (training, test) = populations(8, 850_000, 64, 9);
        for profile in Profile::ALL {
            let outcomes = evaluate_profile(&training, &test, 8, profile);
            assert_eq!(outcomes.len(), test.len());
            assert!(
                outcomes
                    .iter()
                    .all(|row| row.direct_target_protected_disclosures == 0)
            );
        }
    }

    #[test]
    fn strong_peer_signal_materially_improves_exact_cohort_inference() {
        let (train_low, test_low) = populations(32, 550_000, 256, 17);
        let (train_high, test_high) = populations(32, 950_000, 256, 17);
        let low = summarize(&evaluate_profile(
            &train_low,
            &test_low,
            32,
            Profile::ExactCohortPeerMajority,
        ));
        let high = summarize(&evaluate_profile(
            &train_high,
            &test_high,
            32,
            Profile::ExactCohortPeerMajority,
        ));
        assert!(accuracy_ppm(&high) > accuracy_ppm(&low) + 200_000);
    }

    #[test]
    fn randomized_peer_negative_control_stays_near_prior() {
        let (training, test) = populations(8, 850_000, 512, 23);
        let summary = summarize(&evaluate_profile(
            &training,
            &test,
            8,
            Profile::RandomizedPeerNegativeControl,
        ));
        let difference = accuracy_ppm(&summary).abs_diff(prior_accuracy_ppm(&summary));
        assert!(
            difference <= 100_000,
            "accuracy/prior delta was {difference} ppm"
        );
    }

    #[test]
    fn uncorrelated_target_negative_control_stays_near_prior() {
        let (training, test) = populations(8, 850_000, 512, 31);
        let summary = summarize(&evaluate_profile(
            &training,
            &test,
            8,
            Profile::UncorrelatedTargetNegativeControl,
        ));
        let difference = accuracy_ppm(&summary).abs_diff(prior_accuracy_ppm(&summary));
        assert!(
            difference <= 100_000,
            "accuracy/prior delta was {difference} ppm"
        );
    }

    #[test]
    fn independent_audit_matches_outcome_records_not_summary_counters() {
        let (training, test) = populations(16, 850_000, 64, 41);
        for profile in Profile::ALL {
            let outcomes = evaluate_profile(&training, &test, 16, profile);
            let summary = summarize(&outcomes);
            let audit = audit_outcomes(&outcomes);
            assert_eq!(audit.target_count, outcomes.len() as u64);
            assert_eq!(audit.correct_count, summary.correct);
            assert_eq!(audit.direct_target_protected_disclosures, 0);
            assert_eq!(
                audit.direct_target_protected_disclosures,
                summary.direct_target_protected_disclosures
            );
        }
    }

    #[test]
    fn report_has_rectangular_csv_schema_and_audit_fields() {
        let report = render_report();
        let mut lines = report.lines();
        let header = lines.next().expect("header exists");
        let header_width = header.split(',').count();
        assert_eq!(header_width, 20);
        assert!(header.contains("peer_observation_delta_vs_exact_cohort"));
        assert!(header.contains("selective_coverage_ppm"));
        assert!(header.contains("selective_accuracy_ppm"));
        assert!(header.contains("direct_target_protected_disclosures"));
        let mut data_rows = 0;
        for line in lines {
            assert_eq!(line.split(',').count(), header_width, "bad row: {line}");
            data_rows += 1;
        }
        assert_eq!(data_rows, 4 * 5 * Profile::ALL.len() * 5);
    }

    #[test]
    fn selective_risk_coverage_exposes_abstention_tradeoff() {
        let (training, test) = populations(16, 850_000, 64, 47);
        let outcomes = evaluate_profile(&training, &test, 16, Profile::ExactCohortPeerMajority);
        let (low_coverage, _, low_abstentions) = selective_metrics(&outcomes, 500_000);
        assert_eq!(low_coverage, PPM);
        assert_eq!(low_abstentions, 0);

        let mut previous_coverage = PPM;
        for threshold_ppm in CONFIDENCE_THRESHOLDS {
            let (coverage, _, abstentions) = selective_metrics(&outcomes, threshold_ppm);
            assert!(
                coverage <= previous_coverage,
                "coverage increased at threshold {threshold_ppm}"
            );
            let expected_abstentions = outcomes
                .iter()
                .filter(|row| row.confidence_score_ppm < threshold_ppm)
                .count() as u64;
            assert_eq!(abstentions, expected_abstentions);
            previous_coverage = coverage;
        }
    }

    #[test]
    fn privacy_metric_and_peer_evidence_access_are_separate_columns() {
        let (training, test) = populations(8, 850_000, 64, 53);
        let prior = summarize(&evaluate_profile(
            &training,
            &test,
            8,
            Profile::PopulationPrior,
        ));
        let exact = summarize(&evaluate_profile(
            &training,
            &test,
            8,
            Profile::ExactCohortPeerMajority,
        ));
        assert!(prior.peer_observations_used < exact.peer_observations_used);
        assert!(
            prior.peer_observation_delta_vs_exact_cohort
                < exact.peer_observation_delta_vs_exact_cohort
        );

        let coarse = summarize(&evaluate_profile(
            &training,
            &test,
            8,
            Profile::CoarseNeighborhoodPeerMajority,
        ));
        assert!(coarse.peer_observations_used > exact.peer_observations_used);
        assert!(
            coarse.peer_observation_delta_vs_exact_cohort
                > exact.peer_observation_delta_vs_exact_cohort
        );

        let overlapping = summarize(&evaluate_profile(
            &training,
            &test,
            8,
            Profile::OverlappingGroupUnionPeerMajority,
        ));
        assert!(overlapping.peer_observations_used > exact.peer_observations_used);
        assert!(
            overlapping.peer_observation_delta_vs_exact_cohort
                > exact.peer_observation_delta_vs_exact_cohort
        );
    }
}

fn main() {
    let _manifest = MANIFEST;
    print!("{}", render_report());
}
