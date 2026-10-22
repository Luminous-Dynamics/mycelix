use std::collections::BTreeMap;
use std::fmt::Write as FmtWrite;
use std::fs;
use std::io;
use std::path::Path;
use std::{env, process};

const POPULATION_SIZE: usize = 4_096;
const ATTRIBUTE_COUNT: usize = 6;
const COHORT_COUNT: usize = 16;
const COHORT_SIZES: [usize; COHORT_COUNT] = [
    1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610, 987, 1_514,
];
const SEED: u64 = 0xC011_A710_2026_1022;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PopulationKind {
    BalancedIndependent,
    CorrelatedSkewed,
}

impl PopulationKind {
    const ALL: [Self; 2] = [Self::BalancedIndependent, Self::CorrelatedSkewed];

    const fn name(self) -> &'static str {
        match self {
            Self::BalancedIndependent => "balanced_independent",
            Self::CorrelatedSkewed => "correlated_skewed",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DisclosureProfile {
    ExactAttribute,
    PredicateOnly,
}

impl DisclosureProfile {
    const ALL: [Self; 2] = [Self::ExactAttribute, Self::PredicateOnly];

    const fn name(self) -> &'static str {
        match self {
            Self::ExactAttribute => "exact_attribute",
            Self::PredicateOnly => "predicate_only",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Record {
    attributes: [u8; ATTRIBUTE_COUNT],
    // This label is test/evaluator truth only, never part of an observable signature.
    cohort_truth: Option<usize>,
}

fn cohort_attributes(cohort: usize) -> [u8; ATTRIBUTE_COUNT] {
    [
        (cohort % 4) as u8,
        ((cohort / 4) % 4) as u8,
        (cohort % 4) as u8,
        ((cohort / 2) % 4) as u8,
        ((cohort / 4) % 4) as u8,
        (cohort % 3) as u8,
    ]
}

fn task_predicate(attribute: usize, value: u8) -> bool {
    match attribute {
        0 => value >= 2,
        1 => value >= 2,
        2 => value & 1 == 1,
        3 => value >= 2,
        4 => value <= 1,
        5 => value & 1 == 0,
        _ => unreachable!("attribute index is bounded by ATTRIBUTE_COUNT"),
    }
}

fn visible_value(profile: DisclosureProfile, attribute: usize, value: u8) -> u8 {
    match profile {
        DisclosureProfile::ExactAttribute => value,
        DisclosureProfile::PredicateOnly => u8::from(task_predicate(attribute, value)),
    }
}

fn disclosure_signature(
    record: &Record,
    depth: usize,
    profile: DisclosureProfile,
) -> Vec<u8> {
    record
        .attributes
        .iter()
        .enumerate()
        .take(depth)
        .map(|(attribute, &value)| visible_value(profile, attribute, value))
        .collect()
}

fn build_population(kind: PopulationKind) -> Vec<Record> {
    match kind {
        PopulationKind::BalancedIndependent => (0..POPULATION_SIZE)
            .map(|index| Record {
                attributes: std::array::from_fn(|attribute| {
                    ((index >> (attribute * 2)) & 0b11) as u8
                }),
                cohort_truth: None,
            })
            .collect(),
        PopulationKind::CorrelatedSkewed => {
            let mut records = Vec::with_capacity(POPULATION_SIZE);
            for (cohort, &size) in COHORT_SIZES.iter().enumerate() {
                let attributes = cohort_attributes(cohort);
                for _ in 0..size {
                    records.push(Record {
                        attributes,
                        cohort_truth: Some(cohort),
                    });
                }
            }
            records
        }
    }
}

fn candidate_counts(
    records: &[Record],
    depth: usize,
    profile: DisclosureProfile,
) -> Vec<usize> {
    let mut buckets: BTreeMap<Vec<u8>, usize> = BTreeMap::new();
    for record in records {
        *buckets
            .entry(disclosure_signature(record, depth, profile))
            .or_insert(0) += 1;
    }

    records
        .iter()
        .map(|record| {
            *buckets
                .get(&disclosure_signature(record, depth, profile))
                .expect("each record's signature must be in its bucket")
        })
        .collect()
}

fn correlated_cohort_oracle(
    target_cohort: usize,
    depth: usize,
    profile: DisclosureProfile,
) -> usize {
    let target = cohort_attributes(target_cohort);
    let mut candidate_mass = 0;
    for (candidate_cohort, &cohort_size) in COHORT_SIZES.iter().enumerate() {
        let candidate = cohort_attributes(candidate_cohort);
        let mut matches = true;
        for (attribute, (&target_value, &candidate_value)) in
            target.iter().zip(candidate.iter()).enumerate().take(depth)
        {
            if visible_value(profile, attribute, target_value)
                != visible_value(profile, attribute, candidate_value)
            {
                matches = false;
                break;
            }
        }
        if matches {
            candidate_mass += cohort_size;
        }
    }
    candidate_mass
}

fn balanced_candidate_oracle(depth: usize, profile: DisclosureProfile) -> usize {
    match profile {
        DisclosureProfile::ExactAttribute => POPULATION_SIZE / 4_usize.pow(depth as u32),
        DisclosureProfile::PredicateOnly => POPULATION_SIZE / 2_usize.pow(depth as u32),
    }
}

fn nearest_rank(values: &[usize], numerator: usize, denominator: usize) -> usize {
    let rank = (values.len() * numerator).div_ceil(denominator);
    values[rank.saturating_sub(1)]
}

fn rate_permille(numerator: usize, denominator: usize) -> usize {
    if denominator == 0 {
        0
    } else {
        numerator * 1_000 / denominator
    }
}

fn task_utility_permille(
    records: &[Record],
    depth: usize,
    profile: DisclosureProfile,
) -> usize {
    let mut correct = 0_usize;
    let mut total = 0_usize;
    for record in records {
        for (attribute, &original) in record.attributes.iter().enumerate().take(depth) {
            let observed = visible_value(profile, attribute, original);
            let answer = match profile {
                DisclosureProfile::ExactAttribute => task_predicate(attribute, observed),
                DisclosureProfile::PredicateOnly => observed == 1,
            };
            if answer == task_predicate(attribute, original) {
                correct += 1;
            }
            total += 1;
        }
    }
    rate_permille(correct, total)
}

fn fnv_population_fingerprint(records: &[Record]) -> u64 {
    // Non-cryptographic deterministic fingerprint only; CI computes SHA-256 over artifacts.
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for record in records {
        for value in record.attributes {
            hash ^= u64::from(value);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    hash
}

fn summarize(
    records: &[Record],
    population: PopulationKind,
    profile: DisclosureProfile,
    depth: usize,
) -> String {
    let mut sizes = candidate_counts(records, depth, profile);
    sizes.sort_unstable();

    let minimum = sizes[0];
    let p05 = nearest_rank(&sizes, 5, 100);
    let p50 = nearest_rank(&sizes, 50, 100);
    let p90 = nearest_rank(&sizes, 90, 100);
    let p95 = nearest_rank(&sizes, 95, 100);
    let maximum = sizes[sizes.len() - 1];
    let unique = sizes.iter().filter(|&&size| size == 1).count();
    let at_most_10 = sizes.iter().filter(|&&size| size <= 10).count();
    let at_most_100 = sizes.iter().filter(|&&size| size <= 100).count();
    let previous_p50 = if depth == 1 {
        POPULATION_SIZE
    } else {
        let mut previous = candidate_counts(records, depth - 1, profile);
        previous.sort_unstable();
        nearest_rank(&previous, 50, 100)
    };
    let retention = rate_permille(p50, previous_p50);
    let utility = task_utility_permille(records, depth, profile);

    assert_eq!(utility, 1_000, "the disclosure profile must preserve task outcomes");
    format!(
        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        population.name(),
        profile.name(),
        depth,
        minimum,
        p05,
        p50,
        p90,
        p95,
        maximum,
        rate_permille(unique, sizes.len()),
        rate_permille(at_most_10, sizes.len()),
        rate_permille(at_most_100, sizes.len()),
        retention,
        utility,
        sizes.len()
    )
}

fn render_report(populations: &[(PopulationKind, Vec<Record>)]) -> String {
    let mut output = String::new();
    writeln!(&mut output, "# benchmark=life-disclosure-correlated-v1").unwrap();
    writeln!(&mut output, "# seed=0x{SEED:016x}").unwrap();
    writeln!(&mut output, "# population_size={POPULATION_SIZE}").unwrap();
    writeln!(
        &mut output,
        "# correlated_cohort_sizes={}",
        COHORT_SIZES
            .iter()
            .map(usize::to_string)
            .collect::<Vec<_>>()
            .join("|")
    )
    .unwrap();
    for (kind, records) in populations {
        writeln!(
            &mut output,
            "# population_fingerprint_{}_fnv1a64={:016x} (non-cryptographic)",
            kind.name(),
            fnv_population_fingerprint(records)
        )
        .unwrap();
    }
    writeln!(&mut output, "# output_label_policy=no_subject_or_cohort_labels").unwrap();
    writeln!(
        &mut output,
        concat!(
            "population,profile,depth,min_candidates,p05_candidates,p50_candidates,",
            "p90_candidates,p95_candidates,max_candidates,unique_rate_permille,",
            "at_most_10_rate_permille,at_most_100_rate_permille,",
            "candidate_retention_permille,task_utility_accuracy_permille,record_count"
        )
    )
    .unwrap();

    for (kind, records) in populations {
        for profile in DisclosureProfile::ALL {
            for depth in 1..=ATTRIBUTE_COUNT {
                writeln!(
                    &mut output,
                    "{}",
                    summarize(records, *kind, profile, depth)
                )
                .unwrap();
            }
        }
    }
    output
}

fn render_features(records: &[Record]) -> String {
    let mut output = String::from(
        "attribute_0\tattribute_1\tattribute_2\tattribute_3\tattribute_4\tattribute_5\n",
    );
    for record in records {
        writeln!(
            &mut output,
            "{}\t{}\t{}\t{}\t{}\t{}",
            record.attributes[0],
            record.attributes[1],
            record.attributes[2],
            record.attributes[3],
            record.attributes[4],
            record.attributes[5]
        )
        .unwrap();
    }
    output
}

fn write_artifacts(dir: &Path, populations: &[(PopulationKind, Vec<Record>)]) -> io::Result<()> {
    fs::create_dir_all(dir)?;
    for (kind, records) in populations {
        let path = dir.join(format!("{}-features.tsv", kind.name()));
        fs::write(path, render_features(records))?;
    }
    fs::write(dir.join("report.csv"), render_report(populations))?;
    Ok(())
}

fn main() {
    let populations: Vec<(PopulationKind, Vec<Record>)> = PopulationKind::ALL
        .into_iter()
        .map(|kind| (kind, build_population(kind)))
        .collect();
    let args: Vec<String> = env::args().skip(1).collect();
    match args.as_slice() {
        [] => print!("{}", render_report(&populations)),
        [flag, dir] if flag == "--write-corpus-dir" => {
            if let Err(error) = write_artifacts(Path::new(dir), &populations) {
                eprintln!("could not write benchmark artifacts: {error}");
                process::exit(1);
            }
            print!("{}", render_report(&populations));
        }
        _ => {
            eprintln!("usage: life-disclosure-correlated [--write-corpus-dir <directory>]");
            process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cohort_sizes_form_the_declared_skewed_population() {
        assert_eq!(COHORT_SIZES.iter().sum::<usize>(), POPULATION_SIZE);
        let records = build_population(PopulationKind::CorrelatedSkewed);
        assert_eq!(records.len(), POPULATION_SIZE);
        assert_eq!(
            records.iter().filter(|record| record.cohort_truth == Some(0)).count(),
            1
        );
        assert_eq!(
            records
                .iter()
                .filter(|record| record.cohort_truth == Some(COHORT_COUNT - 1))
                .count(),
            COHORT_SIZES[COHORT_COUNT - 1]
        );
    }

    #[test]
    fn balanced_population_matches_closed_form_candidate_oracle() {
        let records = build_population(PopulationKind::BalancedIndependent);
        for profile in DisclosureProfile::ALL {
            for depth in 1..=ATTRIBUTE_COUNT {
                let expected = balanced_candidate_oracle(depth, profile);
                let sizes = candidate_counts(&records, depth, profile);
                assert!(sizes.iter().all(|&size| size == expected));
            }
        }
    }

    #[test]
    fn correlated_population_matches_independent_cohort_enumeration() {
        let records = build_population(PopulationKind::CorrelatedSkewed);
        for profile in DisclosureProfile::ALL {
            for depth in 1..=ATTRIBUTE_COUNT {
                let sizes = candidate_counts(&records, depth, profile);
                for cohort in 0..COHORT_COUNT {
                    let expected = correlated_cohort_oracle(cohort, depth, profile);
                    for (index, record) in records.iter().enumerate() {
                        if record.cohort_truth == Some(cohort) {
                            assert_eq!(
                                sizes[index],
                                expected,
                                "profile={profile:?}, depth={depth}, cohort={cohort}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn both_disclosure_profiles_preserve_every_required_task_answer() {
        for kind in PopulationKind::ALL {
            let records = build_population(kind);
            for record in &records {
                for (attribute, &original) in record.attributes.iter().enumerate() {
                    let exact =
                        visible_value(DisclosureProfile::ExactAttribute, attribute, original);
                    let predicate =
                        visible_value(DisclosureProfile::PredicateOnly, attribute, original);
                    assert_eq!(
                        task_predicate(attribute, exact),
                        predicate == 1,
                        "population={kind:?}, attribute={attribute}"
                    );
                }
            }
        }
    }

    #[test]
    fn candidate_sets_never_expand_as_disclosures_accumulate() {
        for kind in PopulationKind::ALL {
            let records = build_population(kind);
            for profile in DisclosureProfile::ALL {
                let series: Vec<Vec<usize>> = (1..=ATTRIBUTE_COUNT)
                    .map(|depth| candidate_counts(&records, depth, profile))
                    .collect();
                for (depth, pair) in series.windows(2).enumerate() {
                    for (index, (previous, current)) in
                        pair[0].iter().zip(pair[1].iter()).enumerate()
                    {
                        assert!(
                            current <= previous,
                            "candidate set expanded for {kind:?}/{profile:?}, record={index}, depth={}",
                            depth + 2
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn corpus_and_report_are_deterministic_without_truth_labels() {
        let populations: Vec<(PopulationKind, Vec<Record>)> = PopulationKind::ALL
            .into_iter()
            .map(|kind| (kind, build_population(kind)))
            .collect();
        assert_eq!(
            render_report(&populations),
            render_report(&[
                (
                    PopulationKind::BalancedIndependent,
                    build_population(PopulationKind::BalancedIndependent)
                ),
                (
                    PopulationKind::CorrelatedSkewed,
                    build_population(PopulationKind::CorrelatedSkewed)
                )
            ])
        );
        for (_, records) in &populations {
            let first = render_features(records);
            assert_eq!(first, render_features(records));
            let header = first.lines().next().unwrap();
            assert!(!header.contains("cohort"));
            assert!(!header.contains("subject"));
            assert!(!header.contains("truth"));
        }
    }
}
