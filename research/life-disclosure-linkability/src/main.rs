// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

const SUBJECTS: u32 = 120;
const RECIPIENTS: u8 = 3;
const SESSIONS_PER_RECIPIENT: u8 = 2;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum Channel {
    Claims,
    Identifier,
    Proof,
    Envelope,
    Behavior,
}

impl Channel {
    const ALL: [Self; 5] = [
        Self::Claims,
        Self::Identifier,
        Self::Proof,
        Self::Envelope,
        Self::Behavior,
    ];

    const fn label(self) -> &'static str {
        match self {
            Self::Claims => "claims",
            Self::Identifier => "identifier",
            Self::Proof => "proof",
            Self::Envelope => "envelope",
            Self::Behavior => "behavior",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Strategy {
    StableGlobalId,
    PairwiseIdOnly,
    PairwiseIdAndFreshProof,
    MetadataMinimized,
    PredicateOnly,
}

impl Strategy {
    const ALL: [Self; 5] = [
        Self::StableGlobalId,
        Self::PairwiseIdOnly,
        Self::PairwiseIdAndFreshProof,
        Self::MetadataMinimized,
        Self::PredicateOnly,
    ];

    const fn label(self) -> &'static str {
        match self {
            Self::StableGlobalId => "stable_global_id",
            Self::PairwiseIdOnly => "pairwise_id_only",
            Self::PairwiseIdAndFreshProof => "pairwise_id_fresh_proof",
            Self::MetadataMinimized => "metadata_minimized",
            Self::PredicateOnly => "predicate_only",
        }
    }

    const fn has_pairwise_identifier(self) -> bool {
        !matches!(self, Self::StableGlobalId)
    }

    const fn has_fresh_proof(self) -> bool {
        matches!(
            self,
            Self::PairwiseIdAndFreshProof | Self::MetadataMinimized | Self::PredicateOnly
        )
    }

    const fn has_lineage_metadata(self) -> bool {
        matches!(
            self,
            Self::StableGlobalId | Self::PairwiseIdOnly | Self::PairwiseIdAndFreshProof
        )
    }

    const fn has_behavior_metadata(self) -> bool {
        self.has_lineage_metadata()
    }

    const fn uses_quasi_identifying_claims(self) -> bool {
        !matches!(self, Self::PredicateOnly)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Feature {
    channel: Channel,
    key: &'static str,
    value: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Presentation {
    presentation_id: u32,
    // This is benchmark ground truth. It is never included in the attacker-visible
    // presentation feature file.
    synthetic_subject_id: u32,
    recipient_id: u8,
    session_id: u8,
    features: Vec<Feature>,
}

fn add_feature(
    features: &mut Vec<Feature>,
    channel: Channel,
    key: &'static str,
    value: impl Into<String>,
) {
    features.push(Feature {
        channel,
        key,
        value: value.into(),
    });
}

fn make_presentation(strategy: Strategy, subject: u32, recipient: u8, session: u8) -> Presentation {
    let mut features = Vec::new();

    // Two claim profiles deliberately model different disclosure utility/privacy
    // tradeoffs. The quasi-identifier profile uses three broad claims whose
    // combination is unique in this finite synthetic corpus; that is intentional
    // and is documented as content-driven linkability, not a cryptographic break.
    if strategy.uses_quasi_identifying_claims() {
        add_feature(
            &mut features,
            Channel::Claims,
            "age_band",
            format!("age-{}", subject % 3),
        );
        add_feature(
            &mut features,
            Channel::Claims,
            "region_band",
            format!("region-{}", subject % 5),
        );
        add_feature(
            &mut features,
            Channel::Claims,
            "employment_cluster",
            format!("sector-{}", subject % 17),
        );
    } else {
        add_feature(
            &mut features,
            Channel::Claims,
            "adult_eligibility",
            format!("eligible-{}", subject % 2),
        );
    }

    let identifier = if strategy.has_pairwise_identifier() {
        format!("subject-{subject}-recipient-{recipient}")
    } else {
        format!("subject-{subject}")
    };
    add_feature(&mut features, Channel::Identifier, "holder_ref", identifier);

    let presentation_id = subject * u32::from(RECIPIENTS) * u32::from(SESSIONS_PER_RECIPIENT)
        + u32::from(recipient) * u32::from(SESSIONS_PER_RECIPIENT)
        + u32::from(session);

    let proof_ref = if strategy.has_fresh_proof() {
        format!("proof-presentation-{presentation_id}")
    } else {
        format!("proof-subject-{subject}")
    };
    add_feature(&mut features, Channel::Proof, "proof_ref", proof_ref);

    if strategy.has_lineage_metadata() {
        add_feature(
            &mut features,
            Channel::Envelope,
            "lineage_ref",
            format!("lineage-{subject}"),
        );
    }

    if strategy.has_behavior_metadata() {
        add_feature(
            &mut features,
            Channel::Behavior,
            "cadence_class",
            format!("cadence-{}", subject % 4),
        );
    }

    Presentation {
        presentation_id,
        synthetic_subject_id: subject,
        recipient_id: recipient,
        session_id: session,
        features,
    }
}

fn build_corpus(strategy: Strategy) -> Vec<Presentation> {
    let mut presentations = Vec::new();
    for subject in 0..SUBJECTS {
        for recipient in 0..RECIPIENTS {
            for session in 0..SESSIONS_PER_RECIPIENT {
                presentations.push(make_presentation(strategy, subject, recipient, session));
            }
        }
    }
    presentations
}

fn features_in_channel(presentation: &Presentation, channel: Channel) -> Vec<(&str, &str)> {
    let mut features: Vec<_> = presentation
        .features
        .iter()
        .filter(|feature| feature.channel == channel)
        .map(|feature| (feature.key, feature.value.as_str()))
        .collect();
    features.sort_unstable();
    features
}

fn channels_match(left: &Presentation, right: &Presentation, channel: Channel) -> bool {
    let left_features = features_in_channel(left, channel);
    let right_features = features_in_channel(right, channel);
    if left_features.is_empty() || right_features.is_empty() {
        return false;
    }

    if channel == Channel::Claims {
        // The claim-channel attacker links only when the entire disclosed claim
        // vector matches. Individual broad claims may collide, which is measured
        // separately by the predicate-only scenario.
        left_features == right_features
    } else {
        left_features
            .iter()
            .any(|feature| right_features.contains(feature))
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct Metrics {
    positive_pairs: u64,
    negative_pairs: u64,
    true_positive: u64,
    false_positive: u64,
}

impl Metrics {
    fn observe(&mut self, same_subject: bool, predicted_link: bool) {
        if same_subject {
            self.positive_pairs += 1;
            if predicted_link {
                self.true_positive += 1;
            }
        } else {
            self.negative_pairs += 1;
            if predicted_link {
                self.false_positive += 1;
            }
        }
    }

    fn false_negative(self) -> u64 {
        self.positive_pairs - self.true_positive
    }

    fn true_negative(self) -> u64 {
        self.negative_pairs - self.false_positive
    }

    fn precision(self) -> f64 {
        let predicted_positive = self.true_positive + self.false_positive;
        ratio(self.true_positive, predicted_positive)
    }

    fn recall(self) -> f64 {
        ratio(self.true_positive, self.positive_pairs)
    }

    fn false_positive_rate(self) -> f64 {
        ratio(self.false_positive, self.negative_pairs)
    }

    /// AUC for the declared binary exact-match attacker rule: (TPR + TNR) / 2.
    /// This is not an AUC for a trained probabilistic attacker.
    fn binary_rule_auc(self) -> f64 {
        (self.recall() + ratio(self.true_negative(), self.negative_pairs)) / 2.0
    }
}

fn ratio(numerator: u64, denominator: u64) -> f64 {
    if denominator == 0 {
        0.0
    } else {
        numerator as f64 / denominator as f64
    }
}

#[derive(Clone, Debug)]
struct Evaluation {
    by_channel: BTreeMap<Channel, Metrics>,
    any_channel: Metrics,
}

fn evaluate(presentations: &[Presentation]) -> Evaluation {
    let mut by_channel: BTreeMap<Channel, Metrics> = Channel::ALL
        .into_iter()
        .map(|channel| (channel, Metrics::default()))
        .collect();
    let mut any_channel = Metrics::default();

    for (left_index, left) in presentations.iter().enumerate() {
        for right in presentations.iter().skip(left_index + 1) {
            // Cross-recipient pairs only. The benchmark does not count a verifier
            // trivially matching its own repeated session as cross-recipient leakage.
            if left.recipient_id == right.recipient_id {
                continue;
            }

            let same_subject = left.synthetic_subject_id == right.synthetic_subject_id;
            let mut any_match = false;
            for channel in Channel::ALL {
                let matched = channels_match(left, right, channel);
                by_channel
                    .get_mut(&channel)
                    .expect("all channels initialized")
                    .observe(same_subject, matched);
                any_match |= matched;
            }
            any_channel.observe(same_subject, any_match);
        }
    }

    Evaluation {
        by_channel,
        any_channel,
    }
}

fn fnv1a64_fingerprint(presentations: &[Presentation]) -> u64 {
    // Non-cryptographic determinism fingerprint only. The workflow separately
    // hashes the exported synthetic corpus with SHA-256.
    let mut canonical = String::new();
    for presentation in presentations {
        let _ = writeln!(
            canonical,
            "{}\t{}\t{}\t{}",
            presentation.presentation_id,
            presentation.synthetic_subject_id,
            presentation.recipient_id,
            presentation.session_id
        );
        for feature in &presentation.features {
            let _ = writeln!(
                canonical,
                "{}\t{}\t{}",
                feature.channel.label(),
                feature.key,
                feature.value
            );
        }
    }

    let mut hash = 0xcbf29ce484222325_u64;
    for byte in canonical.bytes() {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

fn write_corpus(output_directory: &Path) -> Result<(), Box<dyn Error>> {
    std::fs::create_dir_all(output_directory)?;
    for strategy in Strategy::ALL {
        let presentations = build_corpus(strategy);
        let mut visible_rows = String::from(
            "# synthetic attacker-visible features; subject labels are not included\n",
        );
        let mut truth_rows =
            String::from("# synthetic ground truth; do not feed this file to the attacker\n");

        for presentation in &presentations {
            let _ = writeln!(
                truth_rows,
                "{}\t{}",
                presentation.presentation_id, presentation.synthetic_subject_id
            );
            for feature in &presentation.features {
                let _ = writeln!(
                    visible_rows,
                    "{}\t{}\t{}\t{}\t{}",
                    presentation.presentation_id,
                    presentation.recipient_id,
                    feature.channel.label(),
                    feature.key,
                    feature.value
                );
            }
        }

        std::fs::write(
            output_directory.join(format!("{}-presentations.tsv", strategy.label())),
            visible_rows,
        )?;
        std::fs::write(
            output_directory.join(format!("{}-ground-truth.tsv", strategy.label())),
            truth_rows,
        )?;
    }
    Ok(())
}

fn print_metrics_named(strategy: &str, channel: &str, metrics: Metrics) {
    println!(
        "{},{},{},{},{},{},{},{},{:.6},{:.6},{:.6},{:.6}",
        strategy,
        channel,
        metrics.positive_pairs,
        metrics.negative_pairs,
        metrics.true_positive,
        metrics.false_positive,
        metrics.false_negative(),
        metrics.true_negative(),
        metrics.precision(),
        metrics.recall(),
        metrics.false_positive_rate(),
        metrics.binary_rule_auc()
    );
}

fn print_metrics(strategy: Strategy, channel: &str, metrics: Metrics) {
    print_metrics_named(strategy.label(), channel, metrics);
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9e3779b97f4a7c15);
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d049bb133111eb);
    value ^ (value >> 31)
}

fn random_pair_baseline(presentations: &[Presentation]) -> Metrics {
    // Deterministic pseudo-random negative control, independent of the synthetic
    // subject label and feature values. The modulus approximates the 1/SUBJECTS
    // positive-pair prevalence without reading ground truth.
    let mut metrics = Metrics::default();
    for (left_index, left) in presentations.iter().enumerate() {
        for right in presentations.iter().skip(left_index + 1) {
            if left.recipient_id == right.recipient_id {
                continue;
            }
            let pair_key =
                (u64::from(left.presentation_id) << 32) | u64::from(right.presentation_id);
            let predicted_link =
                splitmix64(pair_key ^ 0xD1CE_BA5E).is_multiple_of(u64::from(SUBJECTS));
            metrics.observe(
                left.synthetic_subject_id == right.synthetic_subject_id,
                predicted_link,
            );
        }
    }
    metrics
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let mut output_directory: Option<PathBuf> = None;
    while let Some(argument) = args.next() {
        if argument == "--write-corpus-dir" {
            let path = args
                .next()
                .ok_or("--write-corpus-dir requires a destination directory")?;
            output_directory = Some(PathBuf::from(path));
        } else {
            return Err(format!("unknown argument: {argument}").into());
        }
    }

    if let Some(directory) = output_directory {
        write_corpus(&directory)?;
        eprintln!("synthetic corpus written to {}", directory.display());
    }

    println!("benchmark=life-disclosure-linkability-v1");
    println!("claim_ceiling=synthetic exact-match attacker; not a production unlinkability proof");
    println!(
        "subjects={},recipients={},sessions_per_recipient={}",
        SUBJECTS, RECIPIENTS, SESSIONS_PER_RECIPIENT
    );
    println!(
        "pairs_scope=cross-recipient only; all strategy outputs are synthetic and deterministic"
    );
    println!(
        "strategy,channel,positive_pairs,negative_pairs,true_positive,false_positive,false_negative,true_negative,precision,recall,false_positive_rate,binary_rule_auc"
    );

    for strategy in Strategy::ALL {
        let presentations = build_corpus(strategy);
        println!(
            "# corpus_fingerprint_fnv1a64_non_cryptographic={},strategy={}",
            fnv1a64_fingerprint(&presentations),
            strategy.label()
        );
        let evaluation = evaluate(&presentations);
        for channel in Channel::ALL {
            print_metrics(
                strategy,
                channel.label(),
                *evaluation
                    .by_channel
                    .get(&channel)
                    .expect("all channels evaluated"),
            );
        }
        print_metrics(strategy, "any_channel", evaluation.any_channel);
    }

    let baseline = build_corpus(Strategy::StableGlobalId);
    print_metrics_named(
        "random_pair_baseline",
        "any_channel",
        random_pair_baseline(&baseline),
    );

    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("error: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn metrics_for(strategy: Strategy, channel: Channel) -> Metrics {
        evaluate(&build_corpus(strategy)).by_channel[&channel]
    }

    #[test]
    fn synthetic_corpus_contains_more_than_one_hundred_thousand_cross_recipient_pairs() {
        let evaluation = evaluate(&build_corpus(Strategy::StableGlobalId));
        let metrics = evaluation.any_channel;
        assert_eq!(metrics.positive_pairs, 1_440);
        assert_eq!(metrics.negative_pairs, 171_360);
        assert_eq!(metrics.positive_pairs + metrics.negative_pairs, 172_800);
    }

    #[test]
    fn stable_global_identifier_links_same_subjects_across_recipients() {
        let metrics = metrics_for(Strategy::StableGlobalId, Channel::Identifier);
        assert_eq!(metrics.true_positive, metrics.positive_pairs);
        assert_eq!(metrics.false_positive, 0);
    }

    #[test]
    fn pairwise_identifiers_do_not_fix_reused_proof_material() {
        let identifiers = metrics_for(Strategy::PairwiseIdOnly, Channel::Identifier);
        let proofs = metrics_for(Strategy::PairwiseIdOnly, Channel::Proof);
        assert_eq!(identifiers.true_positive, 0);
        assert_eq!(identifiers.false_positive, 0);
        assert_eq!(proofs.true_positive, proofs.positive_pairs);
        assert_eq!(proofs.false_positive, 0);
    }

    #[test]
    fn fresh_proof_does_not_fix_stable_lineage_metadata() {
        let proofs = metrics_for(Strategy::PairwiseIdAndFreshProof, Channel::Proof);
        let envelope = metrics_for(Strategy::PairwiseIdAndFreshProof, Channel::Envelope);
        assert_eq!(proofs.true_positive, 0);
        assert_eq!(proofs.false_positive, 0);
        assert_eq!(envelope.true_positive, envelope.positive_pairs);
        assert_eq!(envelope.false_positive, 0);
    }

    #[test]
    fn minimized_metadata_does_not_hide_a_quasi_identifying_claim_vector() {
        let claims = metrics_for(Strategy::MetadataMinimized, Channel::Claims);
        let identifiers = metrics_for(Strategy::MetadataMinimized, Channel::Identifier);
        let proofs = metrics_for(Strategy::MetadataMinimized, Channel::Proof);
        let envelope = metrics_for(Strategy::MetadataMinimized, Channel::Envelope);
        let behavior = metrics_for(Strategy::MetadataMinimized, Channel::Behavior);

        assert_eq!(claims.true_positive, claims.positive_pairs);
        assert_eq!(claims.false_positive, 0);
        assert_eq!(identifiers.true_positive, 0);
        assert_eq!(proofs.true_positive, 0);
        assert_eq!(envelope.true_positive, 0);
        assert_eq!(behavior.true_positive, 0);
    }

    #[test]
    fn broad_predicates_can_create_false_links_even_without_auxiliary_metadata() {
        let claims = metrics_for(Strategy::PredicateOnly, Channel::Claims);
        assert_eq!(claims.true_positive, claims.positive_pairs);
        assert!(claims.false_positive > 0);
        assert!(claims.false_positive_rate() > 0.4);
        assert!(claims.precision() < 0.1);
    }

    #[test]
    fn deterministic_random_pair_control_stays_near_chance() {
        let metrics = random_pair_baseline(&build_corpus(Strategy::StableGlobalId));
        assert_eq!(metrics.positive_pairs, 1_440);
        assert_eq!(metrics.negative_pairs, 171_360);
        assert!((metrics.binary_rule_auc() - 0.5).abs() < 0.01);
        assert!(metrics.recall() < 0.03);
        assert!(metrics.false_positive_rate() < 0.02);
    }

    #[test]
    fn corpus_fingerprint_is_deterministic_and_strategy_specific() {
        let first = build_corpus(Strategy::MetadataMinimized);
        let second = build_corpus(Strategy::MetadataMinimized);
        let baseline = build_corpus(Strategy::StableGlobalId);
        assert_eq!(fnv1a64_fingerprint(&first), fnv1a64_fingerprint(&second));
        assert_ne!(fnv1a64_fingerprint(&first), fnv1a64_fingerprint(&baseline));
    }
}
