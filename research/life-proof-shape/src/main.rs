use std::fmt::Write as FmtWrite;
use std::fs;
use std::io;
use std::path::Path;
use std::{env, process};

const SUBJECTS: usize = 120;
const RECIPIENTS: usize = 3;
const SESSIONS: usize = 2;
const PRESENTATIONS: usize = SUBJECTS * RECIPIENTS * SESSIONS;
const SEED: u64 = 0xBB5_2026_10_22;

#[derive(Clone, Debug, Eq, PartialEq)]
struct Presentation {
    // Ground-truth label is kept in memory only and never exported.
    subject_truth: usize,
    recipient: u8,
    session: u8,
    proof_tag: u64,
    statement_count: u8,
    revealed_index_pattern: u16,
    schema_profile: u8,
    global_id: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Channel {
    FreshProofValue,
    StatementCount,
    RevealedIndexPattern,
    StatementCountAndIndexPattern,
    SchemaProfile,
    GlobalId,
    StructureWithoutGlobalId,
    AllStableFieldsIgnoringFreshProof,
}

impl Channel {
    const ALL: [Self; 8] = [
        Self::FreshProofValue,
        Self::StatementCount,
        Self::RevealedIndexPattern,
        Self::StatementCountAndIndexPattern,
        Self::SchemaProfile,
        Self::GlobalId,
        Self::StructureWithoutGlobalId,
        Self::AllStableFieldsIgnoringFreshProof,
    ];

    const fn name(self) -> &'static str {
        match self {
            Self::FreshProofValue => "fresh_proof_value",
            Self::StatementCount => "statement_count",
            Self::RevealedIndexPattern => "revealed_index_pattern",
            Self::StatementCountAndIndexPattern => "count_plus_index_pattern",
            Self::SchemaProfile => "schema_profile",
            Self::GlobalId => "global_id",
            Self::StructureWithoutGlobalId => "structure_without_global_id",
            Self::AllStableFieldsIgnoringFreshProof => "all_stable_fields_ignoring_proof",
        }
    }

    fn matches(self, left: &Presentation, right: &Presentation) -> bool {
        match self {
            Self::FreshProofValue => left.proof_tag == right.proof_tag,
            Self::StatementCount => left.statement_count == right.statement_count,
            Self::RevealedIndexPattern => {
                left.revealed_index_pattern == right.revealed_index_pattern
            }
            Self::StatementCountAndIndexPattern => {
                left.statement_count == right.statement_count
                    && left.revealed_index_pattern == right.revealed_index_pattern
            }
            Self::SchemaProfile => left.schema_profile == right.schema_profile,
            Self::GlobalId => left.global_id == right.global_id,
            Self::StructureWithoutGlobalId => {
                left.statement_count == right.statement_count
                    && left.revealed_index_pattern == right.revealed_index_pattern
                    && left.schema_profile == right.schema_profile
            }
            Self::AllStableFieldsIgnoringFreshProof => {
                left.statement_count == right.statement_count
                    && left.revealed_index_pattern == right.revealed_index_pattern
                    && left.schema_profile == right.schema_profile
                    && left.global_id == right.global_id
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct Confusion {
    true_positive: usize,
    false_positive: usize,
    false_negative: usize,
    true_negative: usize,
}

impl Confusion {
    fn record(&mut self, same_subject: bool, linked: bool) {
        match (same_subject, linked) {
            (true, true) => self.true_positive += 1,
            (false, true) => self.false_positive += 1,
            (true, false) => self.false_negative += 1,
            (false, false) => self.true_negative += 1,
        }
    }

    fn precision_permille(self) -> usize {
        let predicted_positive = self.true_positive + self.false_positive;
        if predicted_positive == 0 {
            0
        } else {
            self.true_positive * 1_000 / predicted_positive
        }
    }

    fn recall_permille(self) -> usize {
        let actual_positive = self.true_positive + self.false_negative;
        if actual_positive == 0 {
            0
        } else {
            self.true_positive * 1_000 / actual_positive
        }
    }

    fn false_positive_rate_permille(self) -> usize {
        let actual_negative = self.false_positive + self.true_negative;
        if actual_negative == 0 {
            0
        } else {
            self.false_positive * 1_000 / actual_negative
        }
    }
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
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

fn build_corpus() -> Vec<Presentation> {
    let mut records = Vec::with_capacity(PRESENTATIONS);
    for subject in 0..SUBJECTS {
        for recipient in 0..RECIPIENTS {
            for session in 0..SESSIONS {
                let presentation_index =
                    (subject * RECIPIENTS * SESSIONS + recipient * SESSIONS + session) as u64;
                records.push(Presentation {
                    subject_truth: subject,
                    recipient: recipient as u8,
                    session: session as u8,
                    proof_tag: splitmix64(SEED ^ presentation_index),
                    statement_count: 4 + (subject % 5) as u8,
                    revealed_index_pattern: index_pattern(subject),
                    schema_profile: (subject % 2) as u8,
                    global_id: format!("urn:synthetic:subject:{subject:03}"),
                });
            }
        }
    }
    records
}

fn confusion_for(corpus: &[Presentation], channel: Channel) -> Confusion {
    let mut confusion = Confusion::default();
    for (left_index, left) in corpus.iter().enumerate() {
        for right in corpus.iter().skip(left_index + 1) {
            if left.recipient == right.recipient {
                continue;
            }
            let same_subject = left.subject_truth == right.subject_truth;
            confusion.record(same_subject, channel.matches(left, right));
        }
    }
    confusion
}

fn cross_recipient_pair_count(corpus: &[Presentation]) -> usize {
    let mut count = 0;
    for (left_index, left) in corpus.iter().enumerate() {
        for right in corpus.iter().skip(left_index + 1) {
            if left.recipient != right.recipient {
                count += 1;
            }
        }
    }
    count
}

fn fnv_feature_fingerprint(corpus: &[Presentation]) -> u64 {
    // Non-cryptographic deterministic fingerprint only; the workflow adds SHA-256.
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for item in corpus {
        for value in [
            u64::from(item.recipient),
            u64::from(item.session),
            item.proof_tag,
            u64::from(item.statement_count),
            u64::from(item.revealed_index_pattern),
            u64::from(item.schema_profile),
        ] {
            for byte in value.to_le_bytes() {
                hash ^= u64::from(byte);
                hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        for byte in item.global_id.as_bytes() {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    hash
}

fn render_metrics(corpus: &[Presentation]) -> String {
    let mut output = String::new();
    writeln!(&mut output, "# benchmark=life-proof-shape-v1").unwrap();
    writeln!(&mut output, "# seed=0x{SEED:016x}").unwrap();
    writeln!(&mut output, "# subjects={SUBJECTS}").unwrap();
    writeln!(&mut output, "# recipients={RECIPIENTS}").unwrap();
    writeln!(&mut output, "# sessions_per_recipient={SESSIONS}").unwrap();
    writeln!(&mut output, "# presentations={PRESENTATIONS}").unwrap();
    writeln!(
        &mut output,
        "# cross_recipient_pairs={}",
        cross_recipient_pair_count(corpus)
    )
    .unwrap();
    writeln!(
        &mut output,
        "# synthetic_feature_fingerprint_fnv1a64={:016x} (non-cryptographic)",
        fnv_feature_fingerprint(corpus)
    )
    .unwrap();
    writeln!(
        &mut output,
        "# model=exact_equality_attack_per_channel_not_a_trained_attacker"
    )
    .unwrap();
    writeln!(
        &mut output,
        "channel,tp,fp,fn,tn,precision_permille,recall_permille,fpr_permille"
    )
    .unwrap();

    for channel in Channel::ALL {
        let result = confusion_for(corpus, channel);
        writeln!(
            &mut output,
            "{},{},{},{},{},{},{},{}",
            channel.name(),
            result.true_positive,
            result.false_positive,
            result.false_negative,
            result.true_negative,
            result.precision_permille(),
            result.recall_permille(),
            result.false_positive_rate_permille()
        )
        .unwrap();
    }
    output
}

fn opaque_presentation_ref(presentation_id: u32) -> u64 {
    // Domain-separated per-presentation row handle. It is not a subject identifier.
    splitmix64(SEED ^ 0x4f50_4151_5545_5f31 ^ u64::from(presentation_id))
}

fn render_feature_corpus(corpus: &[Presentation]) -> String {
    let mut output = String::from(
        "presentation_ref\trecipient\tsession\tproof_tag\tstatement_count\trevealed_index_pattern\tschema_profile\tglobal_id\n",
    );
    for item in corpus {
        writeln!(
            &mut output,
            "{:016x}\t{}\t{}\t{:016x}\t{}\t{:04x}\t{}\t{}",
            opaque_presentation_ref(item.presentation_id),
            item.recipient,
            item.session,
            item.proof_tag,
            item.statement_count,
            item.revealed_index_pattern,
            item.schema_profile,
            item.global_id
        )
        .unwrap();
    }
    output
}

fn write_corpus(dir: &Path, corpus: &[Presentation]) -> io::Result<()> {
    fs::create_dir_all(dir)?;
    fs::write(
        dir.join("attacker-visible-features.tsv"),
        render_feature_corpus(corpus),
    )?;
    fs::write(dir.join("metrics.csv"), render_metrics(corpus))?;
    Ok(())
}

fn main() {
    let corpus = build_corpus();
    let args: Vec<String> = env::args().skip(1).collect();
    match args.as_slice() {
        [] => print!("{}", render_metrics(&corpus)),
        [flag, dir] if flag == "--write-corpus-dir" => {
            if let Err(error) = write_corpus(Path::new(dir), &corpus) {
                eprintln!("could not write benchmark corpus: {error}");
                process::exit(1);
            }
            print!("{}", render_metrics(&corpus));
        }
        _ => {
            eprintln!("usage: life-proof-shape [--write-corpus-dir <directory>]");
            process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frozen_corpus_has_expected_shape_and_pair_counts() {
        let corpus = build_corpus();
        assert_eq!(corpus.len(), 720);
        assert_eq!(cross_recipient_pair_count(&corpus), 172_800);
        let positives = corpus
            .iter()
            .enumerate()
            .flat_map(|(i, left)| corpus.iter().skip(i + 1).map(move |right| (left, right)))
            .filter(|(left, right)| {
                left.recipient != right.recipient && left.subject_truth == right.subject_truth
            })
            .count();
        assert_eq!(positives, 1_440);
    }

    #[test]
    fn fresh_proof_tags_are_unique_but_do_not_hide_other_channels() {
        let corpus = build_corpus();
        let mut tags: Vec<u64> = corpus.iter().map(|item| item.proof_tag).collect();
        tags.sort_unstable();
        tags.dedup();
        assert_eq!(tags.len(), corpus.len());

        let proof = confusion_for(&corpus, Channel::FreshProofValue);
        assert_eq!(proof.true_positive, 0);
        assert_eq!(proof.false_negative, 1_440);
        assert_eq!(proof.false_positive, 0);

        let ids = confusion_for(&corpus, Channel::GlobalId);
        assert_eq!(ids.true_positive, 1_440);
        assert_eq!(ids.false_positive, 0);
        assert_eq!(ids.false_negative, 0);
    }

    #[test]
    fn structural_fields_create_linkability_and_false_matches() {
        let corpus = build_corpus();
        for channel in [
            Channel::StatementCount,
            Channel::RevealedIndexPattern,
            Channel::StatementCountAndIndexPattern,
            Channel::SchemaProfile,
            Channel::StructureWithoutGlobalId,
        ] {
            let result = confusion_for(&corpus, channel);
            assert!(
                result.true_positive > 0,
                "{channel:?} should retain same-subject matches"
            );
            assert!(
                result.false_positive > 0,
                "{channel:?} is a coarse feature and should create false matches"
            );
        }
    }

    #[test]
    fn ignoring_fresh_proof_bytes_exposes_global_identifier_correlation() {
        let corpus = build_corpus();
        let result = confusion_for(&corpus, Channel::AllStableFieldsIgnoringFreshProof);
        assert_eq!(result.true_positive, 1_440);
        assert_eq!(result.false_positive, 0);
        assert_eq!(result.false_negative, 0);
    }

    #[test]
    fn attacker_channel_features_do_not_depend_on_hidden_ground_truth_labels() {
        let corpus = build_corpus();
        let left = &corpus[0];
        let right = &corpus[1];
        for channel in Channel::ALL {
            let expected = channel.matches(left, right);
            let mut left_without_truth = left.clone();
            let mut right_without_truth = right.clone();
            left_without_truth.subject_truth = usize::MAX;
            right_without_truth.subject_truth = usize::MAX - 1;
            assert_eq!(
                expected,
                channel.matches(&left_without_truth, &right_without_truth)
            );
        }
    }

    #[test]
    fn frozen_attack_channels_match_exact_confusion_matrix_known_answers() {
        let corpus = build_corpus();
        let expected = [
            (
                Channel::FreshProofValue,
                Confusion {
                    true_positive: 0,
                    false_positive: 0,
                    false_negative: 1_440,
                    true_negative: 171_360,
                },
            ),
            (
                Channel::StatementCount,
                Confusion {
                    true_positive: 1_440,
                    false_positive: 33_120,
                    false_negative: 0,
                    true_negative: 138_240,
                },
            ),
            (
                Channel::RevealedIndexPattern,
                Confusion {
                    true_positive: 1_440,
                    false_positive: 20_160,
                    false_negative: 0,
                    true_negative: 151_200,
                },
            ),
            (
                Channel::StatementCountAndIndexPattern,
                Confusion {
                    true_positive: 1_440,
                    false_positive: 2_880,
                    false_negative: 0,
                    true_negative: 168_480,
                },
            ),
            (
                Channel::SchemaProfile,
                Confusion {
                    true_positive: 1_440,
                    false_positive: 84_960,
                    false_negative: 0,
                    true_negative: 86_400,
                },
            ),
            (
                Channel::GlobalId,
                Confusion {
                    true_positive: 1_440,
                    false_positive: 0,
                    false_negative: 0,
                    true_negative: 171_360,
                },
            ),
            (
                Channel::StructureWithoutGlobalId,
                Confusion {
                    true_positive: 1_440,
                    false_positive: 2_880,
                    false_negative: 0,
                    true_negative: 168_480,
                },
            ),
            (
                Channel::AllStableFieldsIgnoringFreshProof,
                Confusion {
                    true_positive: 1_440,
                    false_positive: 0,
                    false_negative: 0,
                    true_negative: 171_360,
                },
            ),
        ];

        for (channel, expected_matrix) in expected {
            assert_eq!(
                confusion_for(&corpus, channel),
                expected_matrix,
                "known answer for channel {channel:?}"
            );
        }
    }

    #[test]
    fn frozen_corpus_fingerprint_matches_the_independent_reference() {
        let corpus = build_corpus();
        assert_eq!(fnv_feature_fingerprint(&corpus), 0x4337_1097_7265_4aa0);
    }

    #[test]
    fn metrics_and_export_are_deterministic_without_truth_or_sequential_ordinals() {
        let corpus = build_corpus();
        assert_eq!(render_metrics(&corpus), render_metrics(&build_corpus()));
        let features = render_feature_corpus(&corpus);
        assert_eq!(features, render_feature_corpus(&build_corpus()));

        let mut rows = features.lines();
        let header = rows.next().unwrap();
        assert!(header.starts_with("presentation_ref\\trecipient\\tsession"));
        assert!(!header.contains("subject_truth"));
        assert!(!header.contains("ground_truth"));
        assert!(!header.contains("subject_index"));
        assert!(!header.contains("presentation_ordinal"));

        let rows: Vec<&str> = features.lines().skip(1).collect();
        assert_eq!(rows.len(), PRESENTATIONS);
        assert!(rows.iter().all(|row| row.split('\\t').count() == 8));

        let mut presentation_refs: Vec<&str> = rows
            .iter()
            .map(|row| row.split('\\t').next().unwrap())
            .collect();
        presentation_refs.sort_unstable();
        presentation_refs.dedup();
        assert_eq!(presentation_refs.len(), PRESENTATIONS);
    }
}
