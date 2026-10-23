use std::fmt::Write as _;

const SUBJECTS: usize = 512;
const SEED: u64 = 0x5EED_C0DE;

#[derive(Clone, Debug, Eq, PartialEq)]
struct Subject {
    statement_count: u8,
    optional_index_mask: u16,
    stable_identifier: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Profile {
    FreshProofOnly,
    ExactStatementCount,
    BucketedStatementCount,
    FixedPaddedStatementCount,
    RevealedIndexPattern,
    FixedPaddedCountPlusRevealedIndices,
    ExactCountPlusRevealedIndices,
    StableDisclosedIdentifier,
}

const PROFILES: [Profile; 8] = [
    Profile::FreshProofOnly,
    Profile::ExactStatementCount,
    Profile::BucketedStatementCount,
    Profile::FixedPaddedStatementCount,
    Profile::RevealedIndexPattern,
    Profile::FixedPaddedCountPlusRevealedIndices,
    Profile::ExactCountPlusRevealedIndices,
    Profile::StableDisclosedIdentifier,
];

impl Profile {
    fn label(self) -> &'static str {
        match self {
            Self::FreshProofOnly => "fresh_proof_only",
            Self::ExactStatementCount => "exact_statement_count",
            Self::BucketedStatementCount => "bucketed_statement_count",
            Self::FixedPaddedStatementCount => "fixed_padded_statement_count",
            Self::RevealedIndexPattern => "revealed_index_pattern",
            Self::FixedPaddedCountPlusRevealedIndices => {
                "fixed_padded_count_plus_revealed_indices"
            }
            Self::ExactCountPlusRevealedIndices => "exact_count_plus_revealed_indices",
            Self::StableDisclosedIdentifier => "stable_disclosed_identifier",
        }
    }

    fn observer_fields(self) -> &'static str {
        match self {
            Self::FreshProofOnly => "none",
            Self::ExactStatementCount => "exact_statement_count",
            Self::BucketedStatementCount => "statement_count_bucket",
            Self::FixedPaddedStatementCount => "common_padded_count",
            Self::RevealedIndexPattern => "optional_index_mask",
            Self::FixedPaddedCountPlusRevealedIndices => {
                "common_padded_count+optional_index_mask"
            }
            Self::ExactCountPlusRevealedIndices => "exact_statement_count+optional_index_mask",
            Self::StableDisclosedIdentifier => "stable_disclosed_identifier",
        }
    }
}

fn next_random(state: &mut u64) -> u64 {
    *state = state
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    *state
}

fn make_optional_index_mask(state: &mut u64) -> u16 {
    let mut mask = 0_u16;
    while mask.count_ones() < 3 {
        let bit = (next_random(state) % 8) as u32;
        mask |= 1_u16 << bit;
    }
    mask
}

fn generate_subjects() -> Vec<Subject> {
    let mut state = SEED;
    (0..SUBJECTS)
        .map(|index| Subject {
            statement_count: 8 + (next_random(&mut state) % 24) as u8,
            optional_index_mask: make_optional_index_mask(&mut state),
            stable_identifier: 0xA11C_E000_0000_0000_u64 | index as u64,
        })
        .collect()
}

fn count_bucket(statement_count: u8) -> u8 {
    (statement_count - 8) / 8
}

fn observable_match(left: &Subject, right: &Subject, profile: Profile) -> bool {
    match profile {
        Profile::FreshProofOnly | Profile::FixedPaddedStatementCount => true,
        Profile::ExactStatementCount => left.statement_count == right.statement_count,
        Profile::BucketedStatementCount => {
            count_bucket(left.statement_count) == count_bucket(right.statement_count)
        }
        Profile::RevealedIndexPattern => left.optional_index_mask == right.optional_index_mask,
        Profile::FixedPaddedCountPlusRevealedIndices => {
            left.optional_index_mask == right.optional_index_mask
        }
        Profile::ExactCountPlusRevealedIndices => {
            left.statement_count == right.statement_count
                && left.optional_index_mask == right.optional_index_mask
        }
        Profile::StableDisclosedIdentifier => left.stable_identifier == right.stable_identifier,
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Metrics {
    unique_candidates: usize,
    at_most_ten_candidates: usize,
    min_candidates: usize,
    p05_candidates: usize,
    p50_candidates: usize,
    p95_candidates: usize,
    max_candidates: usize,
    mean_candidates_milli: usize,
}

fn percentile(sorted: &[usize], percentile: usize) -> usize {
    let rank = (percentile * sorted.len() + 99) / 100;
    sorted[rank.saturating_sub(1)]
}

fn evaluate(subjects: &[Subject], profile: Profile) -> Metrics {
    let candidate_counts: Vec<usize> = subjects
        .iter()
        .map(|target| {
            subjects
                .iter()
                .filter(|candidate| observable_match(target, candidate, profile))
                .count()
        })
        .collect();
    let unique_candidates = candidate_counts.iter().filter(|&&n| n == 1).count();
    let at_most_ten_candidates = candidate_counts.iter().filter(|&&n| n <= 10).count();
    let mut sorted = candidate_counts.clone();
    sorted.sort_unstable();
    let total_candidates: usize = sorted.iter().sum();

    Metrics {
        unique_candidates,
        at_most_ten_candidates,
        min_candidates: sorted[0],
        p05_candidates: percentile(&sorted, 5),
        p50_candidates: percentile(&sorted, 50),
        p95_candidates: percentile(&sorted, 95),
        max_candidates: *sorted.last().expect("subject population is nonempty"),
        mean_candidates_milli: total_candidates * 1_000 / sorted.len(),
    }
}

fn ppm(count: usize, denominator: usize) -> usize {
    count * 1_000_000 / denominator
}

fn render_report(subjects: &[Subject]) -> String {
    let mut report = String::from(
        "profile,observer_fields,subjects,unique_candidate_count,unique_candidate_ppm,at_most_10_candidate_count,at_most_10_candidate_ppm,min_candidates,p05_candidates,p50_candidates,p95_candidates,max_candidates,mean_candidates_milli\n",
    );
    for profile in PROFILES {
        let metrics = evaluate(subjects, profile);
        writeln!(
            report,
            "{},{},{},{},{},{},{},{},{},{},{},{},{}",
            profile.label(),
            profile.observer_fields(),
            subjects.len(),
            metrics.unique_candidates,
            ppm(metrics.unique_candidates, subjects.len()),
            metrics.at_most_ten_candidates,
            ppm(metrics.at_most_ten_candidates, subjects.len()),
            metrics.min_candidates,
            metrics.p05_candidates,
            metrics.p50_candidates,
            metrics.p95_candidates,
            metrics.max_candidates,
            metrics.mean_candidates_milli,
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
    fn fresh_proof_only_preserves_full_candidate_set() {
        let metrics = evaluate(&generate_subjects(), Profile::FreshProofOnly);
        assert_eq!(metrics.min_candidates, SUBJECTS);
        assert_eq!(metrics.p05_candidates, SUBJECTS);
        assert_eq!(metrics.p50_candidates, SUBJECTS);
        assert_eq!(metrics.p95_candidates, SUBJECTS);
        assert_eq!(metrics.max_candidates, SUBJECTS);
        assert_eq!(metrics.unique_candidates, 0);
    }

    #[test]
    fn stable_disclosed_identifier_links_each_synthetic_subject() {
        let metrics = evaluate(&generate_subjects(), Profile::StableDisclosedIdentifier);
        assert_eq!(metrics.unique_candidates, SUBJECTS);
        assert_eq!(metrics.at_most_ten_candidates, SUBJECTS);
        assert_eq!(metrics.min_candidates, 1);
        assert_eq!(metrics.max_candidates, 1);
    }

    #[test]
    fn fixed_padding_removes_count_as_a_matching_feature() {
        let metrics = evaluate(&generate_subjects(), Profile::FixedPaddedStatementCount);
        assert_eq!(metrics.min_candidates, SUBJECTS);
        assert_eq!(metrics.max_candidates, SUBJECTS);
    }

    #[test]
    fn count_bucket_coarsening_never_narrows_candidate_sets() {
        let subjects = generate_subjects();
        for target in &subjects {
            let exact = subjects
                .iter()
                .filter(|candidate| {
                    observable_match(target, candidate, Profile::ExactStatementCount)
                })
                .count();
            let bucketed = subjects
                .iter()
                .filter(|candidate| {
                    observable_match(target, candidate, Profile::BucketedStatementCount)
                })
                .count();
            assert!(bucketed >= exact);
        }
        let exact_metrics = evaluate(&subjects, Profile::ExactStatementCount);
        let bucketed_metrics = evaluate(&subjects, Profile::BucketedStatementCount);
        assert!(bucketed_metrics.p50_candidates > exact_metrics.p50_candidates);
    }

    #[test]
    fn count_padding_does_not_suppress_index_pattern_leakage() {
        let subjects = generate_subjects();
        let padded_count_only = evaluate(&subjects, Profile::FixedPaddedStatementCount);
        let padded_with_indices = evaluate(&subjects, Profile::FixedPaddedCountPlusRevealedIndices);
        let index_only = evaluate(&subjects, Profile::RevealedIndexPattern);
        assert_eq!(padded_count_only.p50_candidates, SUBJECTS);
        assert!(padded_with_indices.p50_candidates < padded_count_only.p50_candidates);
        assert_eq!(padded_with_indices, index_only);
        assert!(index_only.at_most_ten_candidates > 0);
    }

    #[test]
    fn exact_count_and_indices_never_widen_candidate_sets() {
        let subjects = generate_subjects();
        for target in &subjects {
            let exact_count = subjects
                .iter()
                .filter(|candidate| {
                    observable_match(target, candidate, Profile::ExactStatementCount)
                })
                .count();
            let combined = subjects
                .iter()
                .filter(|candidate| {
                    observable_match(target, candidate, Profile::ExactCountPlusRevealedIndices)
                })
                .count();
            assert!(combined <= exact_count);
        }
        let metrics = evaluate(&subjects, Profile::ExactCountPlusRevealedIndices);
        assert!(
            metrics.p50_candidates
                < evaluate(&subjects, Profile::ExactStatementCount).p50_candidates
        );
    }

    #[test]
    fn report_is_deterministic_and_rectangular() {
        let subjects = generate_subjects();
        let first = render_report(&subjects);
        let second = render_report(&subjects);
        assert_eq!(first, second);
        let mut rows = first.lines();
        let header_columns = rows.next().expect("report has header").split(',').count();
        assert_eq!(header_columns, 13);
        for row in rows {
            assert_eq!(row.split(',').count(), header_columns);
        }
        assert_eq!(first.lines().count(), PROFILES.len() + 1);
    }

    #[test]
    fn percentiles_are_monotone() {
        let metrics = evaluate(&generate_subjects(), Profile::BucketedStatementCount);
        assert!(metrics.min_candidates <= metrics.p05_candidates);
        assert!(metrics.p05_candidates <= metrics.p50_candidates);
        assert!(metrics.p50_candidates <= metrics.p95_candidates);
        assert!(metrics.p95_candidates <= metrics.max_candidates);
    }
}
