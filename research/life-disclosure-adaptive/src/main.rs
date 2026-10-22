use std::cmp::Ordering;
use std::fmt::Write as FmtWrite;
use std::fs;
use std::io;
use std::path::Path;
use std::{env, process};

const SUBJECTS: usize = 120;
const RECIPIENTS: usize = 3;
const SESSIONS_PER_RECIPIENT: usize = 2;
const TRAIN_SUBJECTS: usize = 80;
const FEATURE_COUNT: usize = 12;
const NEGATIVE_MULTIPLIER: usize = 5;
const OPTIMIZER_STEPS: usize = 400;
const LEARNING_RATE: f64 = 0.35;
const L2_PENALTY: f64 = 0.015;
const BOOTSTRAP_REPLICATES: usize = 200;
const SEED: u64 = 0xADA9_71CE_2026_1022;

const FEATURE_NAMES: [&str; FEATURE_COUNT] = [
    "claim_0_match",
    "claim_1_match",
    "claim_2_match",
    "claim_vector_match",
    "identifier_match",
    "proof_match",
    "lineage_match",
    "behavior_match",
    "statement_count_match",
    "revealed_index_pattern_match",
    "schema_profile_match",
    "count_and_index_match",
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Strategy {
    StableGlobalId,
    PairwiseIdOnly,
    PairwiseIdFreshProof,
    MetadataMinimized,
    PredicateOnly,
}

impl Strategy {
    const ALL: [Self; 5] = [
        Self::StableGlobalId,
        Self::PairwiseIdOnly,
        Self::PairwiseIdFreshProof,
        Self::MetadataMinimized,
        Self::PredicateOnly,
    ];

    const fn name(self) -> &'static str {
        match self {
            Self::StableGlobalId => "stable_global_id",
            Self::PairwiseIdOnly => "pairwise_id_only",
            Self::PairwiseIdFreshProof => "pairwise_id_fresh_proof",
            Self::MetadataMinimized => "metadata_minimized",
            Self::PredicateOnly => "predicate_only",
        }
    }

    const fn has_quasi_identifying_claims(self) -> bool {
        !matches!(self, Self::PredicateOnly)
    }

    const fn has_global_identifier(self) -> bool {
        matches!(self, Self::StableGlobalId)
    }

    const fn has_reusable_proof(self) -> bool {
        matches!(self, Self::StableGlobalId | Self::PairwiseIdOnly)
    }

    const fn has_lineage_and_behavior(self) -> bool {
        matches!(
            self,
            Self::StableGlobalId | Self::PairwiseIdOnly | Self::PairwiseIdFreshProof
        )
    }
}

#[derive(Clone, Debug)]
struct Presentation {
    presentation_ref: u64,
    subject_truth: usize,
    recipient: usize,
    claims: [Option<u8>; 3],
    identifier: Option<u64>,
    proof_ref: Option<u64>,
    lineage_ref: Option<u64>,
    behavior_class: Option<u8>,
    statement_count: u8,
    revealed_index_pattern: u16,
    schema_profile: u8,
}

#[derive(Clone, Debug)]
struct PairExample {
    pair_ref: u64,
    left_subject: usize,
    right_subject: usize,
    features: [f64; FEATURE_COUNT],
    label: bool,
}

#[derive(Clone, Copy, Debug)]
struct Model {
    intercept: f64,
    weights: [f64; FEATURE_COUNT],
}

#[derive(Clone, Copy, Debug)]
struct Metrics {
    positive_pairs: usize,
    negative_pairs: usize,
    true_positive: usize,
    false_positive: usize,
    false_negative: usize,
    true_negative: usize,
    auc: f64,
    precision_at_half: f64,
    recall_at_half: f64,
    fpr_at_half: f64,
    tpr_at_fpr_one_percent: f64,
    fpr_at_tpr_ninety_five_percent: f64,
}

#[derive(Clone, Copy, Debug)]
struct ScoredPair {
    score: f64,
    label: bool,
    left_subject: usize,
    right_subject: usize,
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

fn presentation_ref(subject: usize, recipient: usize, session: usize) -> u64 {
    let ordinal = subject * RECIPIENTS * SESSIONS_PER_RECIPIENT
        + recipient * SESSIONS_PER_RECIPIENT
        + session;
    splitmix64(SEED ^ 0x5052_4553_5f52_4546 ^ ordinal as u64)
}

fn opaque_pair_ref(left_ref: u64, right_ref: u64) -> u64 {
    let (lower, upper) = if left_ref <= right_ref {
        (left_ref, right_ref)
    } else {
        (right_ref, left_ref)
    };
    splitmix64(SEED ^ 0x5041_4952_5f52_4546 ^ lower.rotate_left(17) ^ upper.rotate_right(11))
}

fn subject_split() -> [bool; SUBJECTS] {
    let mut order: Vec<(u64, usize)> = (0..SUBJECTS)
        .map(|subject| (splitmix64(SEED ^ 0x5350_4c49_545f_5631 ^ subject as u64), subject))
        .collect();
    order.sort_unstable();
    let mut train = [false; SUBJECTS];
    for (_, subject) in order.into_iter().take(TRAIN_SUBJECTS) {
        train[subject] = true;
    }
    train
}

fn index_pattern(subject: usize) -> u16 {
    const PATTERNS: [u16; 8] = [
        0b0000_0011,
        0b0000_1100,
        0b0011_0000,
        0b1100_0000,
        0b0001_0101,
        0b0101_0001,
        0b0010_1010,
        0b1010_0010,
    ];
    PATTERNS[subject % PATTERNS.len()]
}

fn make_presentation(
    strategy: Strategy,
    subject: usize,
    recipient: usize,
    session: usize,
) -> Presentation {
    let claims = if strategy.has_quasi_identifying_claims() {
        [
            Some((subject % 3) as u8),
            Some((subject % 5) as u8),
            Some((subject % 17) as u8),
        ]
    } else {
        [Some(u8::from(subject % 2 == 0)), None, None]
    };

    let identifier = if strategy.has_global_identifier() {
        Some(subject as u64)
    } else {
        Some((subject * RECIPIENTS + recipient) as u64)
    };

    let proof_ref = if strategy.has_reusable_proof() {
        Some(subject as u64)
    } else {
        Some(
            (subject * RECIPIENTS * SESSIONS_PER_RECIPIENT
                + recipient * SESSIONS_PER_RECIPIENT
                + session) as u64,
        )
    };

    let (lineage_ref, behavior_class) = if strategy.has_lineage_and_behavior() {
        (Some(subject as u64), Some((subject % 4) as u8))
    } else {
        (None, None)
    };

    Presentation {
        presentation_ref: presentation_ref(subject, recipient, session),
        subject_truth: subject,
        recipient,
        claims,
        identifier,
        proof_ref,
        lineage_ref,
        behavior_class,
        statement_count: 4 + (subject % 5) as u8,
        revealed_index_pattern: index_pattern(subject),
        schema_profile: (subject % 2) as u8,
    }
}

fn build_presentations(strategy: Strategy) -> Vec<Presentation> {
    let mut presentations = Vec::with_capacity(SUBJECTS * RECIPIENTS * SESSIONS_PER_RECIPIENT);
    for subject in 0..SUBJECTS {
        for recipient in 0..RECIPIENTS {
            for session in 0..SESSIONS_PER_RECIPIENT {
                presentations.push(make_presentation(strategy, subject, recipient, session));
            }
        }
    }
    presentations
}

fn eq_optional<T: PartialEq>(left: Option<T>, right: Option<T>) -> bool {
    match (left, right) {
        (Some(left_value), Some(right_value)) => left_value == right_value,
        _ => false,
    }
}

fn pair_features(left: &Presentation, right: &Presentation) -> [f64; FEATURE_COUNT] {
    let claim_matches: [bool; 3] = std::array::from_fn(|index| {
        eq_optional(left.claims[index], right.claims[index])
    });
    let claim_count_left = left.claims.iter().filter(|claim| claim.is_some()).count();
    let claim_count_right = right.claims.iter().filter(|claim| claim.is_some()).count();
    let claim_vector_match = claim_count_left == claim_count_right
        && claim_count_left > 0
        && left.claims == right.claims;

    let count_match = left.statement_count == right.statement_count;
    let index_match = left.revealed_index_pattern == right.revealed_index_pattern;

    [
        f64::from(u8::from(claim_matches[0])),
        f64::from(u8::from(claim_matches[1])),
        f64::from(u8::from(claim_matches[2])),
        f64::from(u8::from(claim_vector_match)),
        f64::from(u8::from(eq_optional(left.identifier, right.identifier))),
        f64::from(u8::from(eq_optional(left.proof_ref, right.proof_ref))),
        f64::from(u8::from(eq_optional(left.lineage_ref, right.lineage_ref))),
        f64::from(u8::from(eq_optional(left.behavior_class, right.behavior_class))),
        f64::from(u8::from(count_match)),
        f64::from(u8::from(index_match)),
        f64::from(u8::from(left.schema_profile == right.schema_profile)),
        f64::from(u8::from(count_match && index_match)),
    ]
}

fn build_pairs(strategy: Strategy, train_split: bool) -> Vec<PairExample> {
    let presentations = build_presentations(strategy);
    let train_subjects = subject_split();
    let mut pairs = Vec::new();
    for (left_index, left) in presentations.iter().enumerate() {
        for right in presentations.iter().skip(left_index + 1) {
            if left.recipient == right.recipient {
                continue;
            }
            let left_is_train = train_subjects[left.subject_truth];
            let right_is_train = train_subjects[right.subject_truth];
            if left_is_train != right_is_train || left_is_train != train_split {
                continue;
            }
            let label = left.subject_truth == right.subject_truth;
            pairs.push(PairExample {
                pair_ref: opaque_pair_ref(left.presentation_ref, right.presentation_ref),
                left_subject: left.subject_truth,
                right_subject: right.subject_truth,
                features: pair_features(left, right),
                label,
            });
        }
    }
    pairs.sort_unstable_by_key(|pair| pair.pair_ref);
    pairs
}

fn downsample_training_negatives(examples: &[PairExample], strategy: Strategy) -> Vec<PairExample> {
    let positives: Vec<PairExample> = examples.iter().filter(|pair| pair.label).cloned().collect();
    let mut negatives: Vec<PairExample> =
        examples.iter().filter(|pair| !pair.label).cloned().collect();
    negatives.sort_unstable_by_key(|pair| {
        splitmix64(SEED ^ 0x4e45_475f_5341_4d50 ^ pair.pair_ref ^ strategy as u64)
    });
    negatives.truncate(positives.len() * NEGATIVE_MULTIPLIER);

    let mut sampled = positives;
    sampled.extend(negatives);
    sampled.sort_unstable_by_key(|pair| pair.pair_ref);
    sampled
}

fn sigmoid(value: f64) -> f64 {
    if value >= 0.0 {
        1.0 / (1.0 + (-value).exp())
    } else {
        let exp_value = value.exp();
        exp_value / (1.0 + exp_value)
    }
}

impl Model {
    fn score_features(&self, features: &[f64; FEATURE_COUNT]) -> f64 {
        let logit = self
            .weights
            .iter()
            .zip(features.iter())
            .fold(self.intercept, |acc, (weight, feature)| acc + weight * feature);
        sigmoid(logit)
    }

    fn fit(examples: &[PairExample]) -> Self {
        let positives = examples.iter().filter(|pair| pair.label).count();
        let negatives = examples.len() - positives;
        assert!(positives > 0 && negatives > 0, "fit set needs both classes");

        let positive_weight = 0.5 / positives as f64;
        let negative_weight = 0.5 / negatives as f64;
        let mut model = Self {
            intercept: 0.0,
            weights: [0.0; FEATURE_COUNT],
        };

        for _ in 0..OPTIMIZER_STEPS {
            let mut intercept_gradient = 0.0;
            let mut gradients = [0.0; FEATURE_COUNT];
            for pair in examples {
                let prediction = model.score_features(&pair.features);
                let weight = if pair.label {
                    positive_weight
                } else {
                    negative_weight
                };
                let error = prediction - f64::from(u8::from(pair.label));
                intercept_gradient += error * weight;
                for (index, feature) in pair.features.iter().enumerate() {
                    gradients[index] += error * weight * feature;
                }
            }
            for (index, gradient) in gradients.iter_mut().enumerate() {
                *gradient += L2_PENALTY * model.weights[index];
            }
            model.intercept -= LEARNING_RATE * intercept_gradient;
            for (weight, gradient) in model.weights.iter_mut().zip(gradients) {
                *weight -= LEARNING_RATE * gradient;
            }
        }
        model
    }
}

fn binary_rule_score(pair: &PairExample) -> f64 {
    let strong_match = (3..=7).any(|index| pair.features[index] == 1.0);
    f64::from(strong_match)
}

fn score_pairs(examples: &[PairExample], model: Option<&Model>) -> Vec<ScoredPair> {
    examples
        .iter()
        .map(|pair| {
            let score = match model {
                Some(model) => model.score_features(&pair.features),
                None => binary_rule_score(pair),
            };
            ScoredPair {
                score,
                label: pair.label,
                left_subject: pair.left_subject,
                right_subject: pair.right_subject,
            }
        })
        .collect()
}

fn auc_from_weighted_scores(scores: &[ScoredPair], cluster_weights: Option<&[usize; SUBJECTS]>) -> f64 {
    let mut values: Vec<(f64, bool, u64)> = scores
        .iter()
        .filter_map(|pair| {
            let weight = match cluster_weights {
                Some(weights) if pair.left_subject == pair.right_subject => {
                    weights[pair.left_subject] as u64
                }
                Some(weights) => {
                    (weights[pair.left_subject] as u64) * (weights[pair.right_subject] as u64)
                }
                None => 1,
            };
            (weight > 0).then_some((pair.score, pair.label, weight))
        })
        .collect();

    values.sort_unstable_by(|left, right| left.0.total_cmp(&right.0));
    let positive_total: u64 = values
        .iter()
        .filter(|value| value.1)
        .map(|value| value.2)
        .sum();
    let negative_total: u64 = values
        .iter()
        .filter(|value| !value.1)
        .map(|value| value.2)
        .sum();
    if positive_total == 0 || negative_total == 0 {
        return 0.5;
    }

    let mut negatives_below = 0_u64;
    let mut numerator = 0.0_f64;
    let mut start = 0;
    while start < values.len() {
        let score = values[start].0;
        let mut end = start + 1;
        while end < values.len() && values[end].0.total_cmp(&score) == Ordering::Equal {
            end += 1;
        }
        let group_positive: u64 = values[start..end]
            .iter()
            .filter(|value| value.1)
            .map(|value| value.2)
            .sum();
        let group_negative: u64 = values[start..end]
            .iter()
            .filter(|value| !value.1)
            .map(|value| value.2)
            .sum();
        numerator += group_positive as f64
            * (negatives_below as f64 + 0.5 * group_negative as f64);
        negatives_below += group_negative;
        start = end;
    }
    numerator / (positive_total as f64 * negative_total as f64)
}

fn rate_at_threshold(scores: &[ScoredPair], threshold: f64) -> (f64, f64, f64) {
    let mut tp = 0_usize;
    let mut fp = 0_usize;
    let positives = scores.iter().filter(|pair| pair.label).count();
    let negatives = scores.len() - positives;
    for pair in scores {
        if pair.score >= threshold {
            if pair.label {
                tp += 1;
            } else {
                fp += 1;
            }
        }
    }
    let precision = if tp + fp == 0 { 0.0 } else { tp as f64 / (tp + fp) as f64 };
    let recall = if positives == 0 { 0.0 } else { tp as f64 / positives as f64 };
    let fpr = if negatives == 0 { 0.0 } else { fp as f64 / negatives as f64 };
    (precision, recall, fpr)
}

fn operating_points(scores: &[ScoredPair]) -> (f64, f64) {
    let mut sorted = scores.to_vec();
    sorted.sort_unstable_by(|left, right| right.score.total_cmp(&left.score));
    let positives = sorted.iter().filter(|pair| pair.label).count();
    let negatives = sorted.len() - positives;
    if positives == 0 || negatives == 0 {
        return (0.0, 1.0);
    }

    let mut tp = 0_usize;
    let mut fp = 0_usize;
    let mut best_tpr_under_one_percent_fpr = 0.0;
    let mut best_fpr_at_ninety_five_percent_tpr = 1.0;
    let mut start = 0;
    while start < sorted.len() {
        let score = sorted[start].score;
        let mut end = start + 1;
        while end < sorted.len() && sorted[end].score.total_cmp(&score) == Ordering::Equal {
            end += 1;
        }
        for pair in &sorted[start..end] {
            if pair.label {
                tp += 1;
            } else {
                fp += 1;
            }
        }
        let tpr = tp as f64 / positives as f64;
        let fpr = fp as f64 / negatives as f64;
        if fpr <= 0.01 {
            best_tpr_under_one_percent_fpr = best_tpr_under_one_percent_fpr.max(tpr);
        }
        if tpr >= 0.95 {
            best_fpr_at_ninety_five_percent_tpr =
                best_fpr_at_ninety_five_percent_tpr.min(fpr);
        }
        start = end;
    }
    (
        best_tpr_under_one_percent_fpr,
        best_fpr_at_ninety_five_percent_tpr,
    )
}

fn evaluate_scores(scores: &[ScoredPair]) -> Metrics {
    let positives = scores.iter().filter(|pair| pair.label).count();
    let negatives = scores.len() - positives;
    let mut tp = 0;
    let mut fp = 0;
    let mut fn_count = 0;
    let mut tn = 0;
    for pair in scores {
        if pair.score >= 0.5 {
            if pair.label {
                tp += 1;
            } else {
                fp += 1;
            }
        } else if pair.label {
            fn_count += 1;
        } else {
            tn += 1;
        }
    }

    let (precision, recall, fpr) = rate_at_threshold(scores, 0.5);
    let (tpr_at_fpr_one_percent, fpr_at_tpr_ninety_five_percent) = operating_points(scores);
    Metrics {
        positive_pairs: positives,
        negative_pairs: negatives,
        true_positive: tp,
        false_positive: fp,
        false_negative: fn_count,
        true_negative: tn,
        auc: auc_from_weighted_scores(scores, None),
        precision_at_half: precision,
        recall_at_half: recall,
        fpr_at_half: fpr,
        tpr_at_fpr_one_percent,
        fpr_at_tpr_ninety_five_percent,
    }
}

fn bootstrap_auc_ci(scores: &[ScoredPair], test_subjects: &[usize], salt: u64) -> (f64, f64) {
    let mut aucs = Vec::with_capacity(BOOTSTRAP_REPLICATES);
    let mut weights = [0_usize; SUBJECTS];
    for replicate in 0..BOOTSTRAP_REPLICATES {
        weights.fill(0);
        for draw in 0..test_subjects.len() {
            let choice = splitmix64(
                SEED ^ salt ^ ((replicate as u64) << 32) ^ draw as u64,
            ) as usize
                % test_subjects.len();
            weights[test_subjects[choice]] += 1;
        }
        aucs.push(auc_from_weighted_scores(scores, Some(&weights)));
    }
    aucs.sort_unstable_by(f64::total_cmp);
    let lower_index = (aucs.len() * 25 / 1_000).min(aucs.len() - 1);
    let upper_index = (aucs.len() * 975 / 1_000).min(aucs.len() - 1);
    (aucs[lower_index], aucs[upper_index])}

fn candidate_subjects(train_split: bool) -> Vec<usize> {
    let split = subject_split();
    (0..SUBJECTS).filter(|&subject| split[subject] == train_split).collect()
}

fn score_bootstrap_ci(
    scores: &[ScoredPair],
    test_subjects: &[usize],
    salt: u64,
) -> (f64, f64) {
    bootstrap_auc_ci(scores, test_subjects, salt)
}

fn csv_number(value: f64) -> String {
    format!("{value:.6}")
}

fn render_report() -> String {
    let mut output = String::new();
    writeln!(&mut output, "# benchmark=life-disclosure-adaptive-v1").unwrap();
    writeln!(&mut output, "# seed=0x{SEED:016x}").unwrap();
    writeln!(&mut output, "# split=train_test_by_subject_hash_rank").unwrap();
    writeln!(&mut output, "# train_subjects={TRAIN_SUBJECTS}").unwrap();
    writeln!(&mut output, "# test_subjects={}", SUBJECTS - TRAIN_SUBJECTS).unwrap();
    writeln!(&mut output, "# optimizer=balanced_logistic_full_batch_l2").unwrap();
    writeln!(&mut output, "# optimizer_steps={OPTIMIZER_STEPS}").unwrap();
    writeln!(&mut output, "# negative_sampling_ratio=1:{NEGATIVE_MULTIPLIER}").unwrap();
    writeln!(&mut output, "# bootstrap_replicates={BOOTSTRAP_REPLICATES}").unwrap();
    writeln!(&mut output, "# subject_and_presentation_ordinals_not_features=true").unwrap();
    writeln!(
        &mut output,
        "strategy,attacker,train_subjects,test_subjects,train_pairs_fit,train_positive_pairs_fit,train_negative_pairs_fit,test_positive_pairs,test_negative_pairs,auc,auc_ci95_low,auc_ci95_high,precision_at_0_5,recall_at_0_5,fpr_at_0_5,tpr_at_fpr_1pct,fpr_at_tpr_95pct,tp,fp,fn,tn"
    )
    .unwrap();

    for strategy in Strategy::ALL {
        let train_subjects = candidate_subjects(true);
        let test_subjects = candidate_subjects(false);
        let train_all = build_pairs(strategy, true);
        let test_pairs = build_pairs(strategy, false);
        let train_fit = downsample_training_negatives(&train_all, strategy);
        let model = Model::fit(&train_fit);

        let model_scores = score_pairs(&test_pairs, Some(&model));
        let model_metrics = evaluate_scores(&model_scores);
        let model_ci = score_bootstrap_ci(
            &model_scores,
            &test_subjects,
            0x4d4f_4445_4c00 ^ strategy as u64,
        );
        write_metric_row(
            &mut output,
            strategy,
            "balanced_logistic",
            train_subjects.len(),
            test_subjects.len(),
            &train_fit,
            &test_pairs,
            model_metrics,
            model_ci,
        );

        let exact_scores = score_pairs(&test_pairs, None);
        let exact_metrics = evaluate_scores(&exact_scores);
        let exact_ci = score_bootstrap_ci(
            &exact_scores,
            &test_subjects,
            0x4558_4143_5400 ^ strategy as u64,
        );
        write_metric_row(
            &mut output,
            strategy,
            "exact_channel_union",
            train_subjects.len(),
            test_subjects.len(),
            &train_fit,
            &test_pairs,
            exact_metrics,
            exact_ci,
        );
    }
    output
}

fn write_metric_row(
    output: &mut String,
    strategy: Strategy,
    attacker: &str,
    train_subject_count: usize,
    test_subject_count: usize,
    train_fit: &[PairExample],
    test_pairs: &[PairExample],
    metrics: Metrics,
    ci: (f64, f64),
) {
    let train_positive = train_fit.iter().filter(|pair| pair.label).count();
    let train_negative = train_fit.len() - train_positive;
    writeln!(
        output,
        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        strategy.name(),
        attacker,
        train_subject_count,
        test_subject_count,
        train_fit.len(),
        train_positive,
        train_negative,
        metrics.positive_pairs,
        metrics.negative_pairs,
        csv_number(metrics.auc),
        csv_number(ci.0),
        csv_number(ci.1),
        csv_number(metrics.precision_at_half),
        csv_number(metrics.recall_at_half),
        csv_number(metrics.fpr_at_half),
        csv_number(metrics.tpr_at_fpr_one_percent),
        csv_number(metrics.fpr_at_tpr_ninety_five_percent),
        metrics.true_positive,
        metrics.false_positive,
        metrics.false_negative,
        metrics.true_negative
    )
    .unwrap();
    let _ = (test_pairs, _train_subject_count, _test_subject_count);
}

fn render_feature_examples(examples: &[PairExample]) -> String {
    let mut output = String::from("pair_ref");
    for name in FEATURE_NAMES {
        write!(&mut output, "\t{name}").unwrap();
    }
    output.push('\n');

    let mut sorted: Vec<&PairExample> = examples.iter().collect();
    sorted.sort_unstable_by_key(|pair| pair.pair_ref);
    for pair in sorted {
        write!(&mut output, "{:016x}", pair.pair_ref).unwrap();
        for feature in pair.features {
            write!(&mut output, "\t{}", u8::from(feature == 1.0)).unwrap();
        }
        output.push('\n');
    }
    output
}

fn render_pair_labels(examples: &[PairExample]) -> String {
    let mut output = String::from("pair_ref\tlink_label\n");
    let mut sorted: Vec<&PairExample> = examples.iter().collect();
    sorted.sort_unstable_by_key(|pair| pair.pair_ref);
    for pair in sorted {
        writeln!(
            &mut output,
            "{:016x}\t{}",
            pair.pair_ref,
            u8::from(pair.label)
        )
        .unwrap();
    }
    output
}

fn write_artifacts(dir: &Path) -> io::Result<()> {
    fs::create_dir_all(dir)?;
    for strategy in Strategy::ALL {
        let train_all = build_pairs(strategy, true);
        let test_pairs = build_pairs(strategy, false);
        let train_fit = downsample_training_negatives(&train_all, strategy);
        let prefix = strategy.name();
        fs::write(
            dir.join(format!("{prefix}-train-features.tsv")),
            render_feature_examples(&train_fit),
        )?;
        fs::write(
            dir.join(format!("{prefix}-train-labels.tsv")),
            render_pair_labels(&train_fit),
        )?;
        fs::write(
            dir.join(format!("{prefix}-test-features.tsv")),
            render_feature_examples(&test_pairs),
        )?;
        fs::write(
            dir.join(format!("{prefix}-test-labels.tsv")),
            render_pair_labels(&test_pairs),
        )?;
    }
    fs::write(dir.join("metrics.csv"), render_report())?;
    fs::write(
        dir.join("README.txt"),
        "Synthetic pairwise data only. Train/test splits are subject-disjoint. Do not join test labels into model fitting. Features contain only observable equality channels; subject labels and presentation ordinals are stored only in separate label files / internal evaluator. Row references are opaque handles, not identity claims. These are not cryptographic credentials or production attack measurements.\n",
    )?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    match args.as_slice() {
        [] => print!("{}", render_report()),
        [flag, dir] if flag == "--write-corpus-dir" => {
            if let Err(error) = write_artifacts(Path::new(dir)) {
                eprintln!("could not write adaptive benchmark artifacts: {error}");
                process::exit(1);
            }
            print!("{}", render_report());
        }
        _ => {
            eprintln!("usage: life-disclosure-adaptive [--write-corpus-dir <directory>]");
            process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    #[test]
    fn subject_split_is_deterministic_disjoint_and_exact_size() {
        let first = subject_split();
        let second = subject_split();
        assert_eq!(first, second);
        assert_eq!(first.iter().filter(|&&is_train| is_train).count(), TRAIN_SUBJECTS);
        assert_eq!(
            first.iter().filter(|&&is_train| !is_train).count(),
            SUBJECTS - TRAIN_SUBJECTS
        );
    }

    #[test]
    fn train_and_test_pairs_never_share_subject_clusters() {
        let split = subject_split();
        for strategy in Strategy::ALL {
            for (train_split, examples) in [
                (true, build_pairs(strategy, true)),
                (false, build_pairs(strategy, false)),
            ] {
                for pair in examples {
                    assert_eq!(split[pair.left_subject], train_split);
                    assert_eq!(split[pair.right_subject], train_split);
                }
            }
        }
    }

    #[test]
    fn cross_recipient_pair_counts_and_positive_prevalence_are_frozen() {
        for strategy in Strategy::ALL {
            let train = build_pairs(strategy, true);
            let test = build_pairs(strategy, false);
            assert_eq!(train.iter().filter(|pair| pair.label).count(), TRAIN_SUBJECTS * 12);
            assert_eq!(test.iter().filter(|pair| pair.label).count(), (SUBJECTS - TRAIN_SUBJECTS) * 12);
            assert_eq!(train.len(), 76_800);
            assert_eq!(test.len(), 19_200);
        }
    }

    #[test]
    fn feature_vector_has_no_hidden_label_or_sequential_ordinal_field() {
        let a = make_presentation(Strategy::MetadataMinimized, 11, 0, 0);
        let b = make_presentation(Strategy::MetadataMinimized, 11, 1, 0);
        let before = pair_features(&a, &b);
        let mut changed = a.clone();
        changed.subject_truth = 999;
        assert_eq!(before, pair_features(&changed, &b));
        assert_ne!(a.presentation_ref, b.presentation_ref);
    }

    #[test]
    fn every_pair_reference_is_unique_in_a_frozen_partition() {
        let examples = build_pairs(Strategy::MetadataMinimized, false);
        let refs: BTreeSet<u64> = examples.iter().map(|pair| pair.pair_ref).collect();
        assert_eq!(refs.len(), examples.len());
    }

    #[test]
    fn exported_feature_and_label_files_are_separate_and_not_group_ordered() {
        let examples = downsample_training_negatives(
            &build_pairs(Strategy::PairwiseIdFreshProof, true),
            Strategy::PairwiseIdFreshProof,
        );
        let feature_text = render_feature_examples(&examples);
        let label_text = render_pair_labels(&examples);
        assert!(!feature_text.contains("subject_truth"));
        assert!(!feature_text.contains("link_label"));
        assert!(!feature_text.contains("presentation_ordinal"));
        assert!(feature_text.lines().next().unwrap().starts_with("pair_ref\tclaim_0_match"));
        assert_eq!(feature_text.lines().count(), examples.len() + 1);
        assert_eq!(label_text.lines().count(), examples.len() + 1);
        assert_eq!(feature_text, render_feature_examples(&examples));
        assert_eq!(label_text, render_pair_labels(&examples));
    }

    #[test]
    fn logistic_model_is_deterministic_and_emits_finite_scores() {
        let examples = downsample_training_negatives(
            &build_pairs(Strategy::MetadataMinimized, true),
            Strategy::MetadataMinimized,
        );
        let first = Model::fit(&examples);
        let second = Model::fit(&examples);
        assert_eq!(first.intercept.to_bits(), second.intercept.to_bits());
        for (left, right) in first.weights.iter().zip(second.weights) {
            assert_eq!(left.to_bits(), right.to_bits());
        }
        for pair in build_pairs(Strategy::MetadataMinimized, false).iter().take(50) {
            let score = first.score_features(&pair.features);
            assert!(score.is_finite() && (0.0..=1.0).contains(&score));
        }
    }

    #[test]
    fn weighted_auc_and_operating_point_helpers_handle_tied_scores() {
        let scores = [
            ScoredPair { score: 0.9, label: true, left_subject: 80, right_subject: 80 },
            ScoredPair { score: 0.9, label: false, left_subject: 80, right_subject: 81 },
            ScoredPair { score: 0.2, label: false, left_subject: 81, right_subject: 82 },
            ScoredPair { score: 0.1, label: true, left_subject: 82, right_subject: 82 },
        ];
        let auc = auc_from_weighted_scores(&scores, None);
        assert_eq!(auc, 0.375);
        let (tpr, fpr) = operating_points(&scores);
        assert!((0.0..=1.0).contains(&tpr));
        assert!((0.0..=1.0).contains(&fpr));
    }

    #[test]
    fn report_is_deterministic_and_contains_both_attackers_for_each_strategy() {
        let first = render_report();
        assert_eq!(first, render_report());
        for strategy in Strategy::ALL {
            assert!(first.contains(&format!("{},balanced_logistic,", strategy.name())));
            assert!(first.contains(&format!("{},exact_channel_union,", strategy.name())));
        }
    }
}
