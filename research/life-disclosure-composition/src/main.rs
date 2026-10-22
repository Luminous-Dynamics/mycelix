use std::fmt::Write as FmtWrite;

const SUBJECT_COUNT: usize = 4_096;
const ATTRIBUTE_COUNT: usize = 6;
const SAMPLE_COUNT: usize = 128;
const K_THRESHOLD: usize = 10;
const MASK: usize = SUBJECT_COUNT - 1;
const MULTIPLIER: usize = 4_051;
const SEED: u64 = 0x1A2B_3C4D_5E6F_7788;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Profile {
    ExactAttribute,
    PredicateOnly,
}

impl Profile {
    const ALL: [Self; 2] = [Self::ExactAttribute, Self::PredicateOnly];

    const fn name(self) -> &'static str {
        match self {
            Self::ExactAttribute => "exact_attribute",
            Self::PredicateOnly => "predicate_only",
        }
    }
}

fn fields(index: usize) -> [u8; ATTRIBUTE_COUNT] {
    // An odd multiplier is invertible modulo 2^12, permuting the complete 4^6 universe.
    let code = (index * MULTIPLIER + ((SEED as usize) & MASK)) & MASK;
    std::array::from_fn(|attribute| ((code >> (attribute * 2)) & 0b11) as u8)
}

fn required_answer(attribute: usize, value: u8) -> bool {
    match attribute {
        0 => value >= 2,
        1 => value & 1 == 1,
        2 => value >= 2,
        3 => value <= 1,
        4 => value < 2,
        5 => value & 1 == 0,
        _ => unreachable!("attribute index is bounded by ATTRIBUTE_COUNT"),
    }
}

fn observation(profile: Profile, attribute: usize, value: u8) -> u8 {
    match profile {
        Profile::ExactAttribute => value,
        Profile::PredicateOnly => u8::from(required_answer(attribute, value)),
    }
}

fn sequence_matches(
    target: &[u8; ATTRIBUTE_COUNT],
    candidate: &[u8; ATTRIBUTE_COUNT],
    depth: usize,
    profile: Profile,
) -> bool {
    (0..depth).all(|attribute| {
        observation(profile, attribute, target[attribute])
            == observation(profile, attribute, candidate[attribute])
    })
}

fn candidates(target: &[u8; ATTRIBUTE_COUNT], depth: usize, profile: Profile) -> usize {
    (0..SUBJECT_COUNT)
        .filter(|&index| sequence_matches(target, &fields(index), depth, profile))
        .count()
}

fn known_answer(depth: usize, profile: Profile) -> usize {
    match profile {
        Profile::ExactAttribute => SUBJECT_COUNT / 4_usize.pow(depth as u32),
        Profile::PredicateOnly => SUBJECT_COUNT / 2_usize.pow(depth as u32),
    }
}

fn sampled_targets() -> Vec<usize> {
    let offset = (SEED as usize) & MASK;
    (0..SAMPLE_COUNT)
        .map(|sample| (sample * 997 + offset) & MASK)
        .collect()
}

fn fnv_corpus_fingerprint() -> u64 {
    // This is a deterministic fingerprint, not a cryptographic digest.
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for index in 0..SUBJECT_COUNT {
        for value in fields(index) {
            hash ^= u64::from(value);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    hash
}

fn utility_accuracy_permille(targets: &[usize], depth: usize, profile: Profile) -> usize {
    let mut correct = 0_usize;
    let mut total = 0_usize;

    for &index in targets {
        let target = fields(index);
        for (attribute, &value) in target.iter().enumerate().take(depth) {
            let observed = observation(profile, attribute, value);
            let answer = match profile {
                Profile::ExactAttribute => required_answer(attribute, observed),
                Profile::PredicateOnly => observed == 1,
            };
            if answer == required_answer(attribute, value) {
                correct += 1;
            }
            total += 1;
        }
    }

    correct.saturating_mul(1_000).checked_div(total).unwrap_or(1_000)
}

fn nearest_rank(values: &[usize], numerator: usize, denominator: usize) -> usize {
    let rank = (values.len() * numerator).div_ceil(denominator);
    values[rank.saturating_sub(1)]
}

fn render_report() -> String {
    let targets = sampled_targets();
    let mut out = String::new();
    writeln!(&mut out, "# benchmark=life-disclosure-composition-v1").unwrap();
    writeln!(&mut out, "# seed=0x{SEED:016x}").unwrap();
    writeln!(&mut out, "# corpus_size={SUBJECT_COUNT}").unwrap();
    writeln!(&mut out, "# sampled_targets={SAMPLE_COUNT}").unwrap();
    writeln!(
        &mut out,
        "# corpus_fingerprint_fnv1a64={:016x} (non-cryptographic)",
        fnv_corpus_fingerprint()
    )
    .unwrap();
    writeln!(
        &mut out,
        "# composition_assumption=observations_are_known_to_belong_to_one_subject"
    )
    .unwrap();
    writeln!(
        &mut out,
        "profile,depth,min_candidates,p50_candidates,p90_candidates,p95_candidates,max_candidates,unique_rate_permille,at_most_10_rate_permille,candidate_retention_permille,task_utility_accuracy_permille"
    )
    .unwrap();

    for profile in Profile::ALL {
        for depth in 1..=ATTRIBUTE_COUNT {
            let mut sizes: Vec<usize> = targets
                .iter()
                .map(|&index| candidates(&fields(index), depth, profile))
                .collect();
            sizes.sort_unstable();

            let min = sizes[0];
            let max = sizes[sizes.len() - 1];
            let p50 = nearest_rank(&sizes, 50, 100);
            let p90 = nearest_rank(&sizes, 90, 100);
            let p95 = nearest_rank(&sizes, 95, 100);
            let unique = sizes.iter().filter(|&&size| size == 1).count();
            let at_most_k = sizes.iter().filter(|&&size| size <= K_THRESHOLD).count();
            let unique_rate = unique * 1_000 / sizes.len();
            let at_most_k_rate = at_most_k * 1_000 / sizes.len();
            let previous = if depth == 1 {
                SUBJECT_COUNT
            } else {
                known_answer(depth - 1, profile)
            };
            let candidate_retention = p50 * 1_000 / previous;
            let utility = utility_accuracy_permille(&targets, depth, profile);

            assert_eq!(p50, known_answer(depth, profile));
            assert_eq!(utility, 1_000, "required task outcome must not regress");
            writeln!(
                &mut out,
                "{},{depth},{min},{p50},{p90},{p95},{max},{unique_rate},{at_most_k_rate},{candidate_retention},{utility}",
                profile.name()
            )
            .unwrap();
        }
    }
    out
}

fn main() {
    print!("{}", render_report());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn field_mapping_is_a_bijection_over_the_complete_cartesian_corpus() {
        let mut seen = vec![false; SUBJECT_COUNT];
        for index in 0..SUBJECT_COUNT {
            let code = fields(index)
                .iter()
                .enumerate()
                .fold(0_usize, |acc, (i, value)| {
                    acc | (usize::from(*value) << (i * 2))
                });
            assert!(!seen[code], "duplicate tuple for synthetic index {index}");
            seen[code] = true;
        }
        assert!(seen.into_iter().all(|value| value));
    }

    #[test]
    fn each_required_predicate_splits_the_fixture_in_half() {
        let mut positives = [0_usize; ATTRIBUTE_COUNT];
        for index in 0..SUBJECT_COUNT {
            for (attribute, &value) in fields(index).iter().enumerate() {
                positives[attribute] += usize::from(required_answer(attribute, value));
            }
        }
        for (attribute, &positive_count) in positives.iter().enumerate() {
            assert_eq!(
                positive_count,
                SUBJECT_COUNT / 2,
                "attribute {attribute}"
            );
        }
    }

    #[test]
    fn scanned_candidate_sets_match_independent_closed_form_oracle() {
        for index in sampled_targets() {
            let target = fields(index);
            for depth in 1..=ATTRIBUTE_COUNT {
                for profile in Profile::ALL {
                    assert!(sequence_matches(&target, &target, depth, profile));
                    assert_eq!(
                        candidates(&target, depth, profile),
                        known_answer(depth, profile),
                        "profile={profile:?}, depth={depth}, target={index}"
                    );
                }
            }
        }
    }

    #[test]
    fn predicate_only_answers_preserve_every_requested_task_result() {
        for index in 0..SUBJECT_COUNT {
            let tuple = fields(index);
            for (attribute, &value) in tuple.iter().enumerate() {
                let exact = observation(Profile::ExactAttribute, attribute, value);
                let predicate = observation(Profile::PredicateOnly, attribute, value) == 1;
                assert_eq!(
                    required_answer(attribute, exact),
                    predicate,
                    "utility differs at synthetic index {index}, attribute {attribute}"
                );
            }
        }
    }

    #[test]
    fn candidate_sets_never_expand_as_observations_accumulate() {
        for profile in Profile::ALL {
            for index in sampled_targets() {
                let target = fields(index);
                let mut previous = SUBJECT_COUNT;
                for depth in 1..=ATTRIBUTE_COUNT {
                    let current = candidates(&target, depth, profile);
                    assert!(current <= previous);
                    assert_eq!(utility_accuracy_permille(&[index], depth, profile), 1_000);
                    previous = current;
                }
            }
        }
    }

    #[test]
    fn final_depth_is_unique_for_exact_values_but_not_for_predicates() {
        for index in sampled_targets() {
            let target = fields(index);
            assert_eq!(
                candidates(&target, ATTRIBUTE_COUNT, Profile::ExactAttribute),
                1
            );
            assert_eq!(
                candidates(&target, ATTRIBUTE_COUNT, Profile::PredicateOnly),
                64
            );
        }
    }

    #[test]
    fn report_is_reproducible_and_does_not_export_identity_columns() {
        let first = render_report();
        assert_eq!(first, render_report());
        let header = first.lines().find(|line| !line.starts_with('#')).unwrap();
        assert!(!header.contains("subject_id"));
        assert!(!header.contains("target_index"));
        assert!(!header.contains("ground_truth"));
    }
}
