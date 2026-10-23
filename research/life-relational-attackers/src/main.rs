use std::fmt::Write as _;

const PPM: f64 = 1_000_000.0;
const COARSENING_FACTOR: usize = 4;
const TRAIN_COHORTS: usize = 24;
const CALIBRATION_COHORTS: usize = 24;
const TEST_COHORTS: usize = 24;
const EVALUATION_SEEDS: [u64; 5] = [1, 7, 23, 41, 97];
const COHORT_SIZES: [usize; 3] = [4, 8, 16];
const SIGNAL_LEVELS_PPM: [u32; 3] = [650_000, 800_000, 950_000];
const CONFIDENCE_THRESHOLDS_PPM: [u64; 5] = [500_000, 625_000, 750_000, 875_000, 950_000];
const FEATURE_COUNT: usize = 7;
const CALIBRATION_BINS: usize = 10;
const MANIFEST: &str = include_str!("../manifest.json");

#[derive(Clone, Debug, PartialEq, Eq)]
struct Subject {
    subject_id: u64,
    cohort_id: u64,
    coarse_bucket: u64,
    secondary_group_id: u64,
    protected: bool,
    independent_secret: bool,
    peer_proxy: bool,
    randomized_peer_proxy: bool,
}

// Only this type is passed to feature construction and inference. The protected
// label and independent evaluation secret intentionally do not exist here.
#[derive(Clone, Debug, PartialEq, Eq)]
struct AttackerView {
    subject_id: u64,
    cohort_id: u64,
    coarse_bucket: u64,
    secondary_group_id: u64,
    peer_proxy: bool,
    randomized_peer_proxy: bool,
}

impl From<&Subject> for AttackerView {
    fn from(subject: &Subject) -> Self {
        Self {
            subject_id: subject.subject_id,
            cohort_id: subject.cohort_id,
            coarse_bucket: subject.coarse_bucket,
            secondary_group_id: subject.secondary_group_id,
            peer_proxy: subject.peer_proxy,
            randomized_peer_proxy: subject.randomized_peer_proxy,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Profile {
    PopulationPrior,
    ExactCohortMajority,
    CoarseNeighborhoodMajority,
    OverlappingGroupUnionMajority,
    RandomizedPeerControl,
    LogisticExactOnly,
    LogisticCoarseOnly,
    LogisticOverlapOnly,
    LogisticRidge,
    GaussianNaiveBayes,
    UncorrelatedTargetNegativeControl,
}

impl Profile {
    const ALL: [Self; 11] = [
        Self::PopulationPrior,
        Self::ExactCohortMajority,
        Self::CoarseNeighborhoodMajority,
        Self::OverlappingGroupUnionMajority,
        Self::RandomizedPeerControl,
        Self::LogisticExactOnly,
        Self::LogisticCoarseOnly,
        Self::LogisticOverlapOnly,
        Self::LogisticRidge,
        Self::GaussianNaiveBayes,
        Self::UncorrelatedTargetNegativeControl,
    ];

    const fn name(self) -> &'static str {
        match self {
            Self::PopulationPrior => "population_prior",
            Self::ExactCohortMajority => "exact_cohort_majority",
            Self::CoarseNeighborhoodMajority => "coarse_neighborhood_majority",
            Self::OverlappingGroupUnionMajority => "overlapping_group_union_majority",
            Self::RandomizedPeerControl => "randomized_peer_control",
            Self::LogisticExactOnly => "logistic_exact_only_ablation",
            Self::LogisticCoarseOnly => "logistic_coarse_only_ablation",
            Self::LogisticOverlapOnly => "logistic_overlap_only_ablation",
            Self::LogisticRidge => "logistic_ridge",
            Self::GaussianNaiveBayes => "gaussian_naive_bayes",
            Self::UncorrelatedTargetNegativeControl => "uncorrelated_target_negative_control",
        }
    }

    const fn uses_independent_training_target(self) -> bool {
        matches!(self, Self::UncorrelatedTargetNegativeControl)
    }

    const fn uses_fitted_model(self) -> bool {
        matches!(
            self,
            Self::LogisticExactOnly
                | Self::LogisticCoarseOnly
                | Self::LogisticOverlapOnly
                | Self::LogisticRidge
                | Self::GaussianNaiveBayes
                | Self::UncorrelatedTargetNegativeControl
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct FeatureVector([f64; FEATURE_COUNT]);

#[derive(Clone, Debug, PartialEq)]
struct OutcomeRecord {
    truth: bool,
    prediction: bool,
    raw_probability: f64,
    calibrated_probability: f64,
    direct_target_protected_disclosures: u64,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct Summary {
    target_count: u64,
    positive_targets: u64,
    correct: u64,
    true_positive: u64,
    true_negative: u64,
    false_positive: u64,
    false_negative: u64,
    raw_brier: f64,
    calibrated_brier: f64,
    raw_log_loss: f64,
    calibrated_log_loss: f64,
    raw_ece: f64,
    calibrated_ece: f64,
    direct_target_protected_disclosures: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Audit {
    target_count: u64,
    correct_count: u64,
    direct_target_protected_disclosures: u64,
}

#[derive(Clone, Copy, Debug)]
struct DeterministicRng {
    state: u64,
}

impl DeterministicRng {
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
        self.state ^= self.state >> 12;
        self.state ^= self.state << 25;
        self.state ^= self.state >> 27;
        self.state.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }

    fn bit(&mut self) -> bool {
        self.next_u64() & 1 == 1
    }

    fn chance_ppm(&mut self, probability_ppm: u32) -> bool {
        u64::from(probability_ppm) > self.next_u64() % 1_000_000
    }
}

fn derive_seed(seed: u64, cohort_size: usize, signal_ppm: u32, split_tag: u64) -> u64 {
    seed.rotate_left(11)
        ^ (cohort_size as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)
        ^ u64::from(signal_ppm).rotate_left(23)
        ^ split_tag
}

fn generate_split(
    seed: u64,
    cohort_size: usize,
    signal_ppm: u32,
    cohort_count: usize,
    cohort_id_base: u64,
) -> Vec<Subject> {
    let mut rng = DeterministicRng::new(derive_seed(seed, cohort_size, signal_ppm, cohort_id_base));
    let mut subjects = Vec::with_capacity(cohort_count.saturating_mul(cohort_size));
    let secondary_group_count = cohort_count.saturating_div(COARSENING_FACTOR).max(1);
    let secondary_namespace = cohort_id_base.saturating_mul(16);

    for cohort_index in 0..cohort_count {
        let latent = rng.bit();
        let cohort_id = cohort_id_base + cohort_index as u64;
        let coarse_bucket =
            cohort_id_base / COARSENING_FACTOR as u64 + (cohort_index / COARSENING_FACTOR) as u64;

        for member_index in 0..cohort_size {
            let protected = if rng.chance_ppm(signal_ppm) {
                latent
            } else {
                !latent
            };
            let peer_proxy = if rng.chance_ppm(signal_ppm) {
                latent
            } else {
                !latent
            };
            let secondary_group_id = secondary_namespace
                + ((cohort_index * 7 + member_index * 11) % secondary_group_count) as u64;

            subjects.push(Subject {
                subject_id: cohort_id
                    .wrapping_mul(1_000)
                    .wrapping_add(member_index as u64),
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

// Features use only AttackerView fields. For each target, exclude its own row
// and use peer observations reachable through exact, coarse, and overlapping edges.
fn build_features(views: &[AttackerView], cohort_size: usize) -> Vec<FeatureVector> {
    let mut rows = Vec::with_capacity(views.len());
    let all_denominator = views.len().saturating_sub(1).max(1) as f64;
    let exact_denominator = cohort_size.saturating_sub(1).max(1) as f64;

    for (target_index, target) in views.iter().enumerate() {
        let mut exact_count = 0_usize;
        let mut exact_positive = 0_usize;
        let mut coarse_count = 0_usize;
        let mut coarse_positive = 0_usize;
        let mut overlap_count = 0_usize;
        let mut overlap_positive = 0_usize;
        let mut overlap_randomized_positive = 0_usize;

        for (candidate_index, candidate) in views.iter().enumerate() {
            if candidate_index == target_index {
                continue;
            }

            let exact = candidate.cohort_id == target.cohort_id;
            let coarse = candidate.coarse_bucket == target.coarse_bucket;
            let overlap = exact || candidate.secondary_group_id == target.secondary_group_id;

            if exact {
                exact_count += 1;
                exact_positive += candidate.peer_proxy as usize;
            }
            if coarse {
                coarse_count += 1;
                coarse_positive += candidate.peer_proxy as usize;
            }
            if overlap {
                overlap_count += 1;
                overlap_positive += candidate.peer_proxy as usize;
                overlap_randomized_positive += candidate.randomized_peer_proxy as usize;
            }
        }

        let fraction = |positive: usize, count: usize| {
            if count == 0 {
                0.5
            } else {
                positive as f64 / count as f64
            }
        };
        rows.push(FeatureVector([
            fraction(exact_positive, exact_count),
            fraction(coarse_positive, coarse_count),
            fraction(overlap_positive, overlap_count),
            fraction(overlap_randomized_positive, overlap_count),
            exact_count as f64 / exact_denominator,
            coarse_count as f64 / all_denominator,
            overlap_count as f64 / all_denominator,
        ]));
    }

    rows
}

fn training_label(subject: &Subject, profile: Profile) -> bool {
    if profile.uses_independent_training_target() {
        subject.independent_secret
    } else {
        subject.protected
    }
}

fn protected_prevalence(subjects: &[Subject]) -> f64 {
    if subjects.is_empty() {
        0.5
    } else {
        (subjects.iter().filter(|subject| subject.protected).count() as f64 + 1.0)
            / (subjects.len() as f64 + 2.0)
    }
}

fn sigmoid(value: f64) -> f64 {
    if value >= 0.0 {
        1.0 / (1.0 + (-value).exp())
    } else {
        let exp_value = value.exp();
        exp_value / (1.0 + exp_value)
    }
}

fn logit(probability: f64) -> f64 {
    let p = probability.clamp(1.0e-9, 1.0 - 1.0e-9);
    (p / (1.0 - p)).ln()
}

fn dot(weights: &[f64; FEATURE_COUNT], features: &FeatureVector) -> f64 {
    weights
        .iter()
        .zip(features.0.iter())
        .map(|(weight, feature)| weight * feature)
        .sum()
}

#[derive(Clone, Debug)]
struct LogisticModel {
    weights: [f64; FEATURE_COUNT],
    bias: f64,
}

impl LogisticModel {
    fn fit(features: &[FeatureVector], labels: &[bool]) -> Self {
        let mut weights = [0.0; FEATURE_COUNT];
        let positive_count = labels.iter().filter(|label| **label).count();
        let prevalence = (positive_count as f64 + 1.0) / (labels.len() as f64 + 2.0);
        let mut bias = logit(prevalence);
        let rate = 0.25;
        let lambda = 0.04;
        let n = features.len().max(1) as f64;

        for _ in 0..240 {
            let mut weight_gradient = [0.0; FEATURE_COUNT];
            let mut bias_gradient = 0.0;

            for (row, label) in features.iter().zip(labels.iter()) {
                let p = sigmoid(dot(&weights, row) + bias);
                let error = p - (*label as u8 as f64);
                for (gradient, feature) in weight_gradient.iter_mut().zip(row.0.iter()) {
                    *gradient += error * feature;
                }
                bias_gradient += error;
            }

            for (weight, gradient) in weights.iter_mut().zip(weight_gradient.iter()) {
                let derivative = gradient / n + lambda * *weight;
                *weight -= rate * derivative;
            }
            bias -= rate * bias_gradient / n;
        }

        Self { weights, bias }
    }

    fn predict(&self, feature: &FeatureVector) -> f64 {
        sigmoid(dot(&self.weights, feature) + self.bias)
    }
}

#[derive(Clone, Debug)]
struct GaussianNaiveBayesModel {
    means: [[f64; FEATURE_COUNT]; 2],
    variances: [[f64; FEATURE_COUNT]; 2],
    priors: [f64; 2],
}

impl GaussianNaiveBayesModel {
    fn fit(features: &[FeatureVector], labels: &[bool]) -> Self {
        let mut counts = [0_usize; 2];
        let mut sums = [[0.0; FEATURE_COUNT]; 2];
        let mut sums_sq = [[0.0; FEATURE_COUNT]; 2];

        for (feature, label) in features.iter().zip(labels.iter()) {
            let class = *label as usize;
            counts[class] += 1;
            for index in 0..FEATURE_COUNT {
                sums[class][index] += feature.0[index];
                sums_sq[class][index] += feature.0[index] * feature.0[index];
            }
        }

        let mut means = [[0.0; FEATURE_COUNT]; 2];
        let mut variances = [[0.0; FEATURE_COUNT]; 2];
        for class in 0..2 {
            let count = counts[class].max(1) as f64;
            for index in 0..FEATURE_COUNT {
                means[class][index] = sums[class][index] / count;
                let variance =
                    sums_sq[class][index] / count - means[class][index] * means[class][index];
                variances[class][index] = variance.max(0.01);
            }
        }

        let total = features.len() as f64;
        let priors = [
            (counts[0] as f64 + 1.0) / (total + 2.0),
            (counts[1] as f64 + 1.0) / (total + 2.0),
        ];

        Self {
            means,
            variances,
            priors,
        }
    }

    fn predict(&self, feature: &FeatureVector) -> f64 {
        let mut log_scores = [self.priors[0].ln(), self.priors[1].ln()];
        for (class, score) in log_scores.iter_mut().enumerate() {
            for index in 0..FEATURE_COUNT {
                let variance = self.variances[class][index];
                let delta = feature.0[index] - self.means[class][index];
                *score += -0.5 * (2.0 * std::f64::consts::PI * variance).ln()
                    - delta * delta / (2.0 * variance);
            }
        }
        sigmoid(log_scores[1] - log_scores[0])
    }
}

#[derive(Clone, Debug)]
enum FittedModel {
    None,
    Logistic(LogisticModel),
    GaussianNaiveBayes(GaussianNaiveBayesModel),
}

fn features_for_profile(features: &[FeatureVector], profile: Profile) -> Vec<FeatureVector> {
    features
        .iter()
        .map(|feature| {
            let mut selected = [0.0; FEATURE_COUNT];
            match profile {
                Profile::LogisticExactOnly => {
                    selected[0] = feature.0[0];
                    selected[4] = feature.0[4];
                }
                Profile::LogisticCoarseOnly => {
                    selected[1] = feature.0[1];
                    selected[5] = feature.0[5];
                }
                Profile::LogisticOverlapOnly => {
                    selected[2] = feature.0[2];
                    selected[6] = feature.0[6];
                }
                _ => return *feature,
            }
            FeatureVector(selected)
        })
        .collect()
}

fn fit_model(profile: Profile, features: &[FeatureVector], subjects: &[Subject]) -> FittedModel {
    if !profile.uses_fitted_model() {
        return FittedModel::None;
    }

    let labels = subjects
        .iter()
        .map(|subject| training_label(subject, profile))
        .collect::<Vec<_>>();
    match profile {
        Profile::LogisticExactOnly
        | Profile::LogisticCoarseOnly
        | Profile::LogisticOverlapOnly
        | Profile::LogisticRidge
        | Profile::UncorrelatedTargetNegativeControl => {
            FittedModel::Logistic(LogisticModel::fit(features, &labels))
        }
        Profile::GaussianNaiveBayes => {
            FittedModel::GaussianNaiveBayes(GaussianNaiveBayesModel::fit(features, &labels))
        }
        _ => FittedModel::None,
    }
}

fn raw_probability(
    profile: Profile,
    feature: &FeatureVector,
    fitted: &FittedModel,
    prior_probability: f64,
) -> f64 {
    match profile {
        Profile::PopulationPrior => prior_probability,
        Profile::ExactCohortMajority => feature.0[0],
        Profile::CoarseNeighborhoodMajority => feature.0[1],
        Profile::OverlappingGroupUnionMajority => feature.0[2],
        Profile::RandomizedPeerControl => feature.0[3],
        Profile::LogisticExactOnly
        | Profile::LogisticCoarseOnly
        | Profile::LogisticOverlapOnly
        | Profile::LogisticRidge
        | Profile::UncorrelatedTargetNegativeControl => match fitted {
            FittedModel::Logistic(model) => model.predict(feature),
            _ => prior_probability,
        },
        Profile::GaussianNaiveBayes => match fitted {
            FittedModel::GaussianNaiveBayes(model) => model.predict(feature),
            _ => prior_probability,
        },
    }
}

#[derive(Clone, Copy, Debug)]
struct PlattCalibrator {
    slope: f64,
    intercept: f64,
}

impl PlattCalibrator {
    fn fit(raw_probabilities: &[f64], calibration_labels: &[bool]) -> Self {
        let logits = raw_probabilities
            .iter()
            .map(|probability| logit(*probability))
            .collect::<Vec<_>>();
        let labels = calibration_labels
            .iter()
            .map(|label| *label as u8 as f64)
            .collect::<Vec<_>>();
        let n = logits.len().max(1) as f64;
        let mut slope = 1.0;
        let mut intercept = 0.0;

        // Fixed calibration schedule; no test-set access or threshold tuning.
        for _ in 0..240 {
            let mut slope_gradient = 0.0;
            let mut intercept_gradient = 0.0;
            for (x, y) in logits.iter().zip(labels.iter()) {
                let error = sigmoid(slope * x + intercept) - y;
                slope_gradient += error * x;
                intercept_gradient += error;
            }
            slope_gradient = slope_gradient / n + 0.005 * (slope - 1.0);
            intercept_gradient /= n;
            slope = (slope - 0.05 * slope_gradient).clamp(-12.0, 12.0);
            intercept = (intercept - 0.05 * intercept_gradient).clamp(-12.0, 12.0);
        }

        Self { slope, intercept }
    }

    fn apply(self, raw_probability: f64) -> f64 {
        sigmoid(self.slope * logit(raw_probability) + self.intercept)
    }
}

fn metric_brier(probabilities: &[f64], labels: &[bool]) -> f64 {
    mean(
        probabilities
            .iter()
            .zip(labels.iter())
            .map(|(probability, label)| {
                let delta = probability - (*label as u8 as f64);
                delta * delta
            }),
    )
}

fn metric_log_loss(probabilities: &[f64], labels: &[bool]) -> f64 {
    mean(
        probabilities
            .iter()
            .zip(labels.iter())
            .map(|(probability, label)| {
                let p = probability.clamp(1.0e-12, 1.0 - 1.0e-12);
                if *label { -p.ln() } else { -(1.0 - p).ln() }
            }),
    )
}

fn metric_ece(probabilities: &[f64], labels: &[bool]) -> f64 {
    if probabilities.is_empty() {
        return 0.0;
    }
    let mut counts = [0_usize; CALIBRATION_BINS];
    let mut confidence_sums = [0.0; CALIBRATION_BINS];
    let mut label_sums = [0.0; CALIBRATION_BINS];

    for (probability, label) in probabilities.iter().zip(labels.iter()) {
        let bin =
            ((*probability * CALIBRATION_BINS as f64).floor() as usize).min(CALIBRATION_BINS - 1);
        counts[bin] += 1;
        confidence_sums[bin] += probability;
        label_sums[bin] += *label as u8 as f64;
    }

    let total = probabilities.len() as f64;
    let mut ece = 0.0;
    for bin in 0..CALIBRATION_BINS {
        if counts[bin] == 0 {
            continue;
        }
        let count = counts[bin] as f64;
        let mean_confidence = confidence_sums[bin] / count;
        let mean_label = label_sums[bin] / count;
        ece += count / total * (mean_confidence - mean_label).abs();
    }
    ece
}

fn mean(values: impl Iterator<Item = f64>) -> f64 {
    let mut total = 0.0;
    let mut count = 0_u64;
    for value in values {
        total += value;
        count += 1;
    }
    if count == 0 {
        0.0
    } else {
        total / count as f64
    }
}

fn to_ppm(value: f64) -> u64 {
    (value.clamp(0.0, 1.0) * PPM).round() as u64
}

fn to_micro_nats(value: f64) -> u64 {
    (value.max(0.0) * PPM).round() as u64
}

fn probabilities_for(
    profile: Profile,
    features: &[FeatureVector],
    fitted: &FittedModel,
    prior_probability: f64,
) -> Vec<f64> {
    features
        .iter()
        .map(|feature| {
            raw_probability(profile, feature, fitted, prior_probability).clamp(1.0e-9, 1.0 - 1.0e-9)
        })
        .collect()
}

fn evaluate_predictions(raw: &[f64], calibrated: &[f64], truths: &[bool]) -> Vec<OutcomeRecord> {
    raw.iter()
        .zip(calibrated.iter())
        .zip(truths.iter())
        .map(
            |((raw_probability, calibrated_probability), truth)| OutcomeRecord {
                truth: *truth,
                prediction: *calibrated_probability >= 0.5,
                raw_probability: *raw_probability,
                calibrated_probability: *calibrated_probability,
                // The protected truth is evaluator-only and joined after inference/calibration.
                direct_target_protected_disclosures: 0,
            },
        )
        .collect()
}

fn summarize(outcomes: &[OutcomeRecord]) -> Summary {
    let mut summary = Summary {
        target_count: outcomes.len() as u64,
        ..Summary::default()
    };

    for outcome in outcomes {
        summary.positive_targets += outcome.truth as u64;
        summary.correct += (outcome.prediction == outcome.truth) as u64;
        summary.direct_target_protected_disclosures += outcome.direct_target_protected_disclosures;
        match (outcome.truth, outcome.prediction) {
            (true, true) => summary.true_positive += 1,
            (false, false) => summary.true_negative += 1,
            (false, true) => summary.false_positive += 1,
            (true, false) => summary.false_negative += 1,
        }
    }

    let labels = outcomes.iter().map(|row| row.truth).collect::<Vec<_>>();
    let raw = outcomes
        .iter()
        .map(|row| row.raw_probability)
        .collect::<Vec<_>>();
    let calibrated = outcomes
        .iter()
        .map(|row| row.calibrated_probability)
        .collect::<Vec<_>>();
    summary.raw_brier = metric_brier(&raw, &labels);
    summary.calibrated_brier = metric_brier(&calibrated, &labels);
    summary.raw_log_loss = metric_log_loss(&raw, &labels);
    summary.calibrated_log_loss = metric_log_loss(&calibrated, &labels);
    summary.raw_ece = metric_ece(&raw, &labels);
    summary.calibrated_ece = metric_ece(&calibrated, &labels);
    summary
}

fn balanced_accuracy_ppm(summary: &Summary) -> u64 {
    let positives = summary.true_positive + summary.false_negative;
    let negatives = summary.true_negative + summary.false_positive;
    if positives == 0 || negatives == 0 {
        0
    } else {
        let sensitivity = summary.true_positive as f64 / positives as f64;
        let specificity = summary.true_negative as f64 / negatives as f64;
        to_ppm((sensitivity + specificity) / 2.0)
    }
}

fn selective_metrics(outcomes: &[OutcomeRecord], threshold_ppm: u64) -> (u64, u64, u64) {
    let mut accepted = 0_u64;
    let mut correct = 0_u64;
    for outcome in outcomes {
        let confidence = outcome
            .calibrated_probability
            .max(1.0 - outcome.calibrated_probability);
        if to_ppm(confidence) >= threshold_ppm {
            accepted += 1;
            correct += (outcome.prediction == outcome.truth) as u64;
        }
    }
    (
        if outcomes.is_empty() {
            0
        } else {
            accepted * 1_000_000 / outcomes.len() as u64
        },
        correct
            .saturating_mul(1_000_000)
            .checked_div(accepted)
            .unwrap_or(0),
        outcomes.len() as u64 - accepted,
    )
}

fn audit_outcomes(outcomes: &[OutcomeRecord]) -> Audit {
    let mut audit = Audit::default();
    for outcome in outcomes {
        audit.target_count += 1;
        audit.correct_count += (outcome.prediction == outcome.truth) as u64;
        audit.direct_target_protected_disclosures += outcome.direct_target_protected_disclosures;
    }
    audit
}

fn run_profile(
    profile: Profile,
    train_subjects: &[Subject],
    train_features: &[FeatureVector],
    calibration_subjects: &[Subject],
    calibration_features: &[FeatureVector],
    test_subjects: &[Subject],
    test_features: &[FeatureVector],
) -> (Vec<OutcomeRecord>, Summary, Audit) {
    let model_train_features = features_for_profile(train_features, profile);
    let model_calibration_features = features_for_profile(calibration_features, profile);
    let model_test_features = features_for_profile(test_features, profile);
    let fitted = fit_model(profile, &model_train_features, train_subjects);
    let prior_probability = protected_prevalence(train_subjects);

    let calibration_raw = probabilities_for(
        profile,
        &model_calibration_features,
        &fitted,
        prior_probability,
    );
    // The calibration map is fit on disjoint calibration communities and protected labels.
    let calibration_truth = calibration_subjects
        .iter()
        .map(|subject| subject.protected)
        .collect::<Vec<_>>();
    let calibrator = PlattCalibrator::fit(&calibration_raw, &calibration_truth);

    let test_raw = probabilities_for(profile, &model_test_features, &fitted, prior_probability);
    let test_calibrated = test_raw
        .iter()
        .map(|probability| calibrator.apply(*probability))
        .collect::<Vec<_>>();
    let test_truth = test_subjects
        .iter()
        .map(|subject| subject.protected)
        .collect::<Vec<_>>();

    // Attacker prediction and calibration are complete before evaluator truth is joined.
    let outcomes = evaluate_predictions(&test_raw, &test_calibrated, &test_truth);
    let summary = summarize(&outcomes);
    let audit = audit_outcomes(&outcomes);
    (outcomes, summary, audit)
}

fn render_report() -> String {
    debug_assert!(!MANIFEST.is_empty());
    let mut output = String::new();
    writeln!(
        &mut output,
        "profile,seed_id,cohort_size,peer_signal_ppm,test_targets,positive_targets,accuracy_ppm,balanced_accuracy_ppm,false_positive_rate_ppm,false_negative_rate_ppm,raw_brier_ppm,calibrated_brier_ppm,raw_log_loss_micro_nats,calibrated_log_loss_micro_nats,raw_ece_ppm,calibrated_ece_ppm,direct_target_protected_disclosures,audit_target_count,audit_correct_count,audit_direct_target_protected_disclosures,decision_threshold_ppm,selective_coverage_ppm,selective_accuracy_ppm,abstained_targets"
    )
    .expect("write to string");

    for seed in EVALUATION_SEEDS {
        for cohort_size in COHORT_SIZES {
            for signal_ppm in SIGNAL_LEVELS_PPM {
                let train_subjects =
                    generate_split(seed, cohort_size, signal_ppm, TRAIN_COHORTS, 0);
                let calibration_subjects = generate_split(
                    seed,
                    cohort_size,
                    signal_ppm,
                    CALIBRATION_COHORTS,
                    1_000_000,
                );
                let test_subjects =
                    generate_split(seed, cohort_size, signal_ppm, TEST_COHORTS, 2_000_000);
                let train_views = train_subjects
                    .iter()
                    .map(AttackerView::from)
                    .collect::<Vec<_>>();
                let calibration_views = calibration_subjects
                    .iter()
                    .map(AttackerView::from)
                    .collect::<Vec<_>>();
                let test_views = test_subjects
                    .iter()
                    .map(AttackerView::from)
                    .collect::<Vec<_>>();

                let train_features = build_features(&train_views, cohort_size);
                let calibration_features = build_features(&calibration_views, cohort_size);
                let test_features = build_features(&test_views, cohort_size);

                for profile in Profile::ALL {
                    let (outcomes, summary, audit) = run_profile(
                        profile,
                        &train_subjects,
                        &train_features,
                        &calibration_subjects,
                        &calibration_features,
                        &test_subjects,
                        &test_features,
                    );
                    let positives = summary.true_positive + summary.false_negative;
                    let negatives = summary.true_negative + summary.false_positive;
                    let fpr = if negatives == 0 {
                        0
                    } else {
                        to_ppm(summary.false_positive as f64 / negatives as f64)
                    };
                    let fnr = if positives == 0 {
                        0
                    } else {
                        to_ppm(summary.false_negative as f64 / positives as f64)
                    };

                    for threshold_ppm in CONFIDENCE_THRESHOLDS_PPM {
                        let (coverage_ppm, selective_accuracy_ppm, abstained) =
                            selective_metrics(&outcomes, threshold_ppm);
                        writeln!(
                            &mut output,
                            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                            profile.name(),
                            seed,
                            cohort_size,
                            signal_ppm,
                            summary.target_count,
                            summary.positive_targets,
                            summary
                                .correct
                                .saturating_mul(1_000_000)
                                .checked_div(summary.target_count)
                                .unwrap_or(0),
                            balanced_accuracy_ppm(&summary),
                            fpr,
                            fnr,
                            to_ppm(summary.raw_brier),
                            to_ppm(summary.calibrated_brier),
                            to_micro_nats(summary.raw_log_loss),
                            to_micro_nats(summary.calibrated_log_loss),
                            to_ppm(summary.raw_ece),
                            to_ppm(summary.calibrated_ece),
                            summary.direct_target_protected_disclosures,
                            audit.target_count,
                            audit.correct_count,
                            audit.direct_target_protected_disclosures,
                            threshold_ppm,
                            coverage_ppm,
                            selective_accuracy_ppm,
                            abstained,
                        )
                        .expect("write to string");
                    }
                }
            }
        }
    }

    output
}

fn main() {
    let _manifest = MANIFEST;
    print!("{}", render_report());
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    fn make_split(seed: u64, cohort_size: usize, signal_ppm: u32, base: u64) -> Vec<Subject> {
        generate_split(seed, cohort_size, signal_ppm, 24, base)
    }

    fn fit_and_predict(
        profile: Profile,
        seed: u64,
        signal_ppm: u32,
    ) -> (Vec<OutcomeRecord>, Summary) {
        let train = make_split(seed, 8, signal_ppm, 0);
        let calibration = make_split(seed, 8, signal_ppm, 1_000_000);
        let test = make_split(seed, 8, signal_ppm, 2_000_000);
        let train_views = train.iter().map(AttackerView::from).collect::<Vec<_>>();
        let calibration_views = calibration
            .iter()
            .map(AttackerView::from)
            .collect::<Vec<_>>();
        let test_views = test.iter().map(AttackerView::from).collect::<Vec<_>>();
        let train_features = build_features(&train_views, 8);
        let calibration_features = build_features(&calibration_views, 8);
        let test_features = build_features(&test_views, 8);
        let (outcomes, summary, _) = run_profile(
            profile,
            &train,
            &train_features,
            &calibration,
            &calibration_features,
            &test,
            &test_features,
        );
        (outcomes, summary)
    }

    fn manifest_integer_array(key: &str) -> Vec<u64> {
        let marker = format!("\"{key}\": [");
        let section = MANIFEST
            .split(marker.as_str())
            .nth(1)
            .expect("manifest array must exist")
            .split(']')
            .next()
            .expect("manifest array must close");
        section
            .split(',')
            .filter_map(|value| value.trim().parse::<u64>().ok())
            .collect()
    }

    #[test]
    fn manifest_freezes_profiles_seeds_signals_and_thresholds() {
        assert!(MANIFEST.contains("attacker_profiles"));
        for profile in Profile::ALL {
            assert!(
                MANIFEST.contains(profile.name()),
                "missing {}",
                profile.name()
            );
        }
        assert_eq!(
            manifest_integer_array("evaluation_seeds"),
            EVALUATION_SEEDS.to_vec()
        );
        assert_eq!(
            manifest_integer_array("cohort_sizes"),
            COHORT_SIZES
                .iter()
                .map(|value| *value as u64)
                .collect::<Vec<_>>()
        );
        assert_eq!(
            manifest_integer_array("peer_signal_probability_ppm"),
            SIGNAL_LEVELS_PPM
                .iter()
                .map(|value| u64::from(*value))
                .collect::<Vec<_>>()
        );
        assert_eq!(
            manifest_integer_array("decision_threshold_ppm"),
            CONFIDENCE_THRESHOLDS_PPM.to_vec()
        );
    }

    #[test]
    fn train_calibration_and_test_communities_are_disjoint() {
        let train = make_split(7, 8, 800_000, 0);
        let calibration = make_split(7, 8, 800_000, 1_000_000);
        let test = make_split(7, 8, 800_000, 2_000_000);
        let ids = |subjects: &[Subject]| {
            subjects
                .iter()
                .map(|s| s.cohort_id)
                .collect::<BTreeSet<_>>()
        };
        let train_ids = ids(&train);
        let calibration_ids = ids(&calibration);
        let test_ids = ids(&test);
        assert!(train_ids.is_disjoint(&calibration_ids));
        assert!(train_ids.is_disjoint(&test_ids));
        assert!(calibration_ids.is_disjoint(&test_ids));
    }

    #[test]
    fn attacker_views_exclude_truth_and_hidden_label_flips_do_not_change_features() {
        let test = make_split(23, 8, 800_000, 2_000_000);
        let mut flipped = test.clone();
        for subject in &mut flipped {
            subject.protected = !subject.protected;
            subject.independent_secret = !subject.independent_secret;
        }
        let views_a = test.iter().map(AttackerView::from).collect::<Vec<_>>();
        let views_b = flipped.iter().map(AttackerView::from).collect::<Vec<_>>();
        assert_eq!(views_a, views_b);
        assert_eq!(build_features(&views_a, 8), build_features(&views_b, 8));
    }

    #[test]
    fn relation_ablation_profiles_expose_only_their_declared_features() {
        let features = [FeatureVector([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])];
        assert_eq!(
            features_for_profile(&features, Profile::LogisticExactOnly)[0],
            FeatureVector([0.1, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0])
        );
        assert_eq!(
            features_for_profile(&features, Profile::LogisticCoarseOnly)[0],
            FeatureVector([0.0, 0.2, 0.0, 0.0, 0.0, 0.6, 0.0])
        );
        assert_eq!(
            features_for_profile(&features, Profile::LogisticOverlapOnly)[0],
            FeatureVector([0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.7])
        );
    }

    #[test]
    fn strong_signal_improves_exact_cohort_attack_vs_weak_signal() {
        let (low, _) = fit_and_predict(Profile::ExactCohortMajority, 23, 650_000);
        let (high, _) = fit_and_predict(Profile::ExactCohortMajority, 23, 950_000);
        let accuracy = |outcomes: &[OutcomeRecord]| {
            outcomes
                .iter()
                .filter(|row| row.prediction == row.truth)
                .count() as f64
                / outcomes.len() as f64
        };
        assert!(
            accuracy(&high) > accuracy(&low) + 0.10,
            "high={} low={}",
            accuracy(&high),
            accuracy(&low)
        );
    }

    #[test]
    fn independent_secret_control_has_no_hidden_truth_feature() {
        let (_, summary) = fit_and_predict(Profile::UncorrelatedTargetNegativeControl, 41, 800_000);
        let accuracy_ppm = summary.correct.saturating_mul(1_000_000) / summary.target_count;
        assert!(
            accuracy_ppm.abs_diff(500_000) <= 150_000,
            "uncorrelated control accuracy ppm={accuracy_ppm}"
        );
    }

    #[test]
    fn independent_audit_recomputes_summary_from_outcome_records() {
        let (outcomes, summary) = fit_and_predict(Profile::LogisticRidge, 97, 800_000);
        let audit = audit_outcomes(&outcomes);
        assert_eq!(audit.target_count, outcomes.len() as u64);
        assert_eq!(audit.correct_count, summary.correct);
        assert_eq!(audit.direct_target_protected_disclosures, 0);
        assert_eq!(
            audit.direct_target_protected_disclosures,
            summary.direct_target_protected_disclosures
        );
    }

    #[test]
    fn selective_coverage_is_monotone_and_abstentions_are_exact() {
        let (outcomes, _) = fit_and_predict(Profile::GaussianNaiveBayes, 7, 800_000);
        let mut prior_coverage = 1_000_000;
        for threshold in CONFIDENCE_THRESHOLDS_PPM {
            let (coverage, _, abstained) = selective_metrics(&outcomes, threshold);
            assert!(coverage <= prior_coverage);
            let expected = outcomes
                .iter()
                .filter(|row| {
                    to_ppm(
                        row.calibrated_probability
                            .max(1.0 - row.calibrated_probability),
                    ) < threshold
                })
                .count() as u64;
            assert_eq!(abstained, expected);
            prior_coverage = coverage;
        }
    }

    #[test]
    fn report_is_deterministic_rectangular_and_has_all_calibration_metrics() {
        let a = render_report();
        let b = render_report();
        assert_eq!(a, b);
        let mut lines = a.lines();
        let header = lines.next().expect("header");
        let width = header.split(',').count();
        assert_eq!(width, 24);
        for field in [
            "raw_brier_ppm",
            "calibrated_brier_ppm",
            "raw_log_loss_micro_nats",
            "calibrated_log_loss_micro_nats",
            "raw_ece_ppm",
            "calibrated_ece_ppm",
            "false_positive_rate_ppm",
            "false_negative_rate_ppm",
            "selective_coverage_ppm",
        ] {
            assert!(header.contains(field));
        }
        let rows = lines.collect::<Vec<_>>();
        assert_eq!(
            rows.len(),
            EVALUATION_SEEDS.len()
                * COHORT_SIZES.len()
                * SIGNAL_LEVELS_PPM.len()
                * Profile::ALL.len()
                * CONFIDENCE_THRESHOLDS_PPM.len()
        );
        for row in rows {
            assert_eq!(row.split(',').count(), width, "bad row {row}");
        }
    }
}
