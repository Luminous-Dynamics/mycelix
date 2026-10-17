use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Partition {
    A,
    B,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConnectionGoal {
    Dating,
    Friendship,
    ActivityPartner,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct InterestWeight {
    pub tag: u16,
    pub weight: u8,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Participant {
    pub id: u32,
    pub partition: Partition,
    pub age: u8,
    pub minimum_age: u8,
    pub maximum_age: u8,
    pub goal: ConnectionGoal,
    pub interests: Vec<InterestWeight>,
    pub minimum_shared_interests: usize,
    pub minimum_utility: u32,
    pub active: bool,
    pub blocked_ids: Vec<u32>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RankingPolicy {
    OneSidedUtility,
    MutualUtility,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Recommendation {
    pub viewer_id: u32,
    pub candidate_id: u32,
    pub viewer_utility: u32,
    pub candidate_utility: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EvaluationMetrics {
    pub feasible_pair_count: usize,
    pub eligible_candidate_users: usize,
    pub exposed_candidate_users: usize,
    pub candidate_coverage: f64,
    pub exposure_gini: f64,
    pub recommendation_count: usize,
    pub mutual_shortlist_pair_count: usize,
    pub invalid_recommendation_count: usize,
}

/// Validate the benchmark input and fail closed on malformed profiles.
pub fn validate_market(participants: &[Participant]) -> Result<(), String> {
    if participants.len() < 2 {
        return Err("market must contain at least two participants".to_string());
    }

    let mut ids = BTreeSet::new();
    let mut partitions = BTreeSet::new();

    for person in participants {
        if !ids.insert(person.id) {
            return Err(format!("duplicate participant id: {}", person.id));
        }
        partitions.insert(person.partition);

        if !(18..=120).contains(&person.age)
            || !(18..=120).contains(&person.minimum_age)
            || !(18..=120).contains(&person.maximum_age)
            || person.minimum_age > person.maximum_age
        {
            return Err(format!("invalid adult age or age range for participant {}", person.id));
        }

        if person.minimum_shared_interests > person.interests.len() {
            return Err(format!(
                "minimum_shared_interests exceeds declared interests for participant {}",
                person.id
            ));
        }

        let mut tags = BTreeSet::new();
        for interest in &person.interests {
            if !(1..=5).contains(&interest.weight) {
                return Err(format!("interest weight must be in 1..=5 for participant {}", person.id));
            }
            if !tags.insert(interest.tag) {
                return Err(format!("duplicate interest tag for participant {}", person.id));
            }
        }
    }

    if partitions.len() != 2 {
        return Err("benchmark requires both abstract partitions A and B".to_string());
    }

    Ok(())
}

fn shared_interest_count(a: &Participant, b: &Participant) -> usize {
    a.interests
        .iter()
        .filter(|left| b.interests.iter().any(|right| right.tag == left.tag))
        .count()
}

/// A viewer-specific, synthetic utility based only on declared shared-interest weights.
/// This is not a psychological or real-world compatibility score.
pub fn directed_utility(viewer: &Participant, candidate: &Participant) -> u32 {
    viewer
        .interests
        .iter()
        .filter(|left| candidate.interests.iter().any(|right| right.tag == left.tag))
        .map(|interest| u32::from(interest.weight) * 10)
        .sum()
}

/// Pair eligibility is symmetric even though utility is viewer-specific.
/// Call only with validated benchmark data when using it as a release gate.
pub fn eligible_pair(a: &Participant, b: &Participant) -> bool {
    if a.id == b.id
        || a.partition == b.partition
        || !a.active
        || !b.active
        || a.goal != b.goal
        || b.age < a.minimum_age
        || b.age > a.maximum_age
        || a.age < b.minimum_age
        || a.age > b.maximum_age
        || a.blocked_ids.contains(&b.id)
        || b.blocked_ids.contains(&a.id)
    {
        return false;
    }

    let shared = shared_interest_count(a, b);
    if shared < a.minimum_shared_interests || shared < b.minimum_shared_interests {
        return false;
    }

    directed_utility(a, b) >= a.minimum_utility
        && directed_utility(b, a) >= b.minimum_utility
}

/// Return top-k private recommendations for each viewer. No shared match or chat
/// is created here; these rows are evaluation observations only.
pub fn recommend(
    participants: &[Participant],
    top_k: usize,
    policy: RankingPolicy,
) -> Result<Vec<Recommendation>, String> {
    validate_market(participants)?;

    let by_id: BTreeMap<u32, &Participant> =
        participants.iter().map(|person| (person.id, person)).collect();
    let mut viewers: Vec<&Participant> = participants
        .iter()
        .filter(|person| person.active)
        .collect();
    viewers.sort_by_key(|person| person.id);

    let mut output = Vec::new();

    for viewer in viewers {
        let mut ranked = Vec::new();

        for candidate in participants.iter().filter(|candidate| candidate.id != viewer.id) {
            if !eligible_pair(viewer, candidate) {
                continue;
            }

            ranked.push(Recommendation {
                viewer_id: viewer.id,
                candidate_id: candidate.id,
                viewer_utility: directed_utility(viewer, candidate),
                candidate_utility: directed_utility(candidate, viewer),
            });
        }

        ranked.sort_by(|left, right| {
            let left_person = by_id.get(&left.candidate_id).expect("validated candidate exists");
            let right_person = by_id.get(&right.candidate_id).expect("validated candidate exists");

            match policy {
                RankingPolicy::OneSidedUtility => right
                    .viewer_utility
                    .cmp(&left.viewer_utility)
                    .then_with(|| left.candidate_id.cmp(&right.candidate_id)),
                RankingPolicy::MutualUtility => {
                    let left_floor = left.viewer_utility.min(left.candidate_utility);
                    let right_floor = right.viewer_utility.min(right.candidate_utility);
                    let left_sum = left.viewer_utility + left.candidate_utility;
                    let right_sum = right.viewer_utility + right.candidate_utility;

                    right_floor
                        .cmp(&left_floor)
                        .then_with(|| right_sum.cmp(&left_sum))
                        .then_with(|| left_person.id.cmp(&right_person.id))
                }
            }
        });

        ranked.truncate(top_k);
        output.extend(ranked);
    }

    Ok(output)
}

fn eligible_candidate_pool(participants: &[Participant]) -> (BTreeSet<u32>, usize) {
    let mut eligible_people = BTreeSet::new();
    let mut feasible_pairs = 0usize;

    for left_index in 0..participants.len() {
        for right_index in (left_index + 1)..participants.len() {
            let left = &participants[left_index];
            let right = &participants[right_index];

            if eligible_pair(left, right) {
                feasible_pairs += 1;
                eligible_people.insert(left.id);
                eligible_people.insert(right.id);
            }
        }
    }

    (eligible_people, feasible_pairs)
}

fn gini(values: &[usize]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }

    let sum: usize = values.iter().sum();
    if sum == 0 {
        return 0.0;
    }

    let mut pairwise_difference = 0.0f64;
    for left in values {
        for right in values {
            pairwise_difference += left.abs_diff(*right) as f64;
        }
    }

    pairwise_difference / (2.0 * values.len() as f64 * sum as f64)
}

/// Evaluate recommendation exposure. Candidate coverage and exposure Gini are
/// calculated only over participants who have at least one feasible counterpart.
pub fn evaluate(
    participants: &[Participant],
    recommendations: &[Recommendation],
) -> EvaluationMetrics {
    let (eligible_people, feasible_pair_count) = eligible_candidate_pool(participants);
    let by_id: BTreeMap<u32, &Participant> =
        participants.iter().map(|person| (person.id, person)).collect();
    let mut exposure: BTreeMap<u32, usize> =
        eligible_people.iter().map(|id| (*id, 0usize)).collect();
    let mut invalid_recommendation_count = 0usize;
    let mut directed_pairs = BTreeSet::new();

    for row in recommendations {
        directed_pairs.insert((row.viewer_id, row.candidate_id));

        match (by_id.get(&row.viewer_id), by_id.get(&row.candidate_id)) {
            (Some(viewer), Some(candidate)) if eligible_pair(viewer, candidate) => {
                if let Some(count) = exposure.get_mut(&row.candidate_id) {
                    *count += 1;
                }
            }
            _ => invalid_recommendation_count += 1,
        }
    }

    let exposed_candidate_users = exposure.values().filter(|count| **count > 0).count();
    let values: Vec<usize> = exposure.values().copied().collect();
    let candidate_coverage = if eligible_people.is_empty() {
        0.0
    } else {
        exposed_candidate_users as f64 / eligible_people.len() as f64
    };

    let mut mutual_shortlist_pair_count = 0usize;
    for &(viewer_id, candidate_id) in &directed_pairs {
        if viewer_id < candidate_id && directed_pairs.contains(&(candidate_id, viewer_id)) {
            mutual_shortlist_pair_count += 1;
        }
    }

    EvaluationMetrics {
        feasible_pair_count,
        eligible_candidate_users: eligible_people.len(),
        exposed_candidate_users,
        candidate_coverage,
        exposure_gini: gini(&values),
        recommendation_count: recommendations.len(),
        mutual_shortlist_pair_count,
        invalid_recommendation_count,
    }
}

/// Generate deterministic synthetic profiles. The two partitions are abstract
/// matching sides only; they do not encode gender, sexuality, or real identities.
pub fn synthetic_market(seed: u64, participant_count: usize) -> Result<Vec<Participant>, String> {
    if participant_count < 4 || participant_count % 2 != 0 {
        return Err("participant_count must be even and at least 4".to_string());
    }

    let mut rng = Rng::new(seed);
    let mut people = Vec::with_capacity(participant_count);
    let midpoint = participant_count / 2;

    for index in 0..participant_count {
        let age = rng.range(21, 60) as u8;
        let minimum_age = age.saturating_sub(rng.range(0, 12) as u8).max(18);
        let maximum_age = age.saturating_add(rng.range(0, 15) as u8).min(75);
        let interest_count = rng.range(4, 9);
        let mut interests = Vec::with_capacity(interest_count);

        while interests.len() < interest_count {
            let tag = rng.range(0, 23) as u16;
            if interests.iter().all(|interest: &InterestWeight| interest.tag != tag) {
                interests.push(InterestWeight {
                    tag,
                    weight: rng.range(1, 5) as u8,
                });
            }
        }
        interests.sort_by_key(|interest| interest.tag);

        let goal = match rng.range(0, 2) {
            0 => ConnectionGoal::Dating,
            1 => ConnectionGoal::Friendship,
            _ => ConnectionGoal::ActivityPartner,
        };

        people.push(Participant {
            id: index as u32,
            partition: if index < midpoint { Partition::A } else { Partition::B },
            age,
            minimum_age,
            maximum_age,
            goal,
            interests,
            minimum_shared_interests: rng.range(1, 3),
            minimum_utility: rng.range(10, 60) as u32,
            active: rng.range(0, 99) < 95,
            blocked_ids: Vec::new(),
        });
    }

    for index in 0..participant_count {
        let block_count = rng.range(0, 3);
        while people[index].blocked_ids.len() < block_count {
            let other_id = rng.range(0, participant_count - 1) as u32;
            if other_id != index as u32 && !people[index].blocked_ids.contains(&other_id) {
                people[index].blocked_ids.push(other_id);
            }
        }
        people[index].blocked_ids.sort_unstable();
    }

    validate_market(&people)?;
    Ok(people)
}

/// Stable non-cryptographic fingerprint for detecting same-input changes in this
/// dependency-free experiment. It is not a security or provenance digest.
pub fn corpus_fingerprint_fnv1a64(participants: &[Participant]) -> String {
    let mut ordered: Vec<&Participant> = participants.iter().collect();
    ordered.sort_by_key(|person| person.id);
    let mut canonical = String::new();

    for person in ordered {
        let partition = match person.partition {
            Partition::A => "A",
            Partition::B => "B",
        };
        let goal = match person.goal {
            ConnectionGoal::Dating => "dating",
            ConnectionGoal::Friendship => "friendship",
            ConnectionGoal::ActivityPartner => "activity",
        };

        canonical.push_str(&format!(
            "{}|{}|{}|{}|{}|{}|{}|{}|{}|",
            person.id,
            partition,
            person.age,
            person.minimum_age,
            person.maximum_age,
            goal,
            person.minimum_shared_interests,
            person.minimum_utility,
            person.active
        ));

        let mut interests = person.interests.clone();
        interests.sort_by_key(|interest| interest.tag);
        for interest in interests {
            canonical.push_str(&format!("{}:{},", interest.tag, interest.weight));
        }
        canonical.push('|');

        let mut blocked = person.blocked_ids.clone();
        blocked.sort_unstable();
        for blocked_id in blocked {
            canonical.push_str(&format!("{},", blocked_id));
        }
        canonical.push('\n');
    }

    let mut hash = 0xcbf29ce484222325u64;
    for byte in canonical.bytes() {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }

    format!("{hash:016x}")
}

struct Rng {
    state: u64,
}

impl Rng {
    fn new(seed: u64) -> Self {
        let normalized_seed = if seed == 0 { 0x9e3779b97f4a7c15 } else { seed };
        Self { state: normalized_seed }
    }

    fn next_u64(&mut self) -> u64 {
        let mut value = self.state;
        value ^= value << 13;
        value ^= value >> 7;
        value ^= value << 17;
        self.state = value;
        value
    }

    fn range(&mut self, minimum: usize, maximum: usize) -> usize {
        debug_assert!(minimum <= maximum);
        minimum + (self.next_u64() % (maximum - minimum + 1) as u64) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn person(
        id: u32,
        partition: Partition,
        age: u8,
        minimum_age: u8,
        maximum_age: u8,
        goal: ConnectionGoal,
        interests: &[(u16, u8)],
    ) -> Participant {
        Participant {
            id,
            partition,
            age,
            minimum_age,
            maximum_age,
            goal,
            interests: interests
                .iter()
                .map(|(tag, weight)| InterestWeight { tag: *tag, weight: *weight })
                .collect(),
            minimum_shared_interests: 1,
            minimum_utility: 10,
            active: true,
            blocked_ids: Vec::new(),
        }
    }

    #[test]
    fn synthetic_corpus_and_recommendations_are_deterministic() {
        let first = synthetic_market(42, 100).expect("valid synthetic corpus");
        let second = synthetic_market(42, 100).expect("same seed is valid");
        assert_eq!(first, second);

        let first_result =
            recommend(&first, 5, RankingPolicy::MutualUtility).expect("valid market");
        let second_result =
            recommend(&second, 5, RankingPolicy::MutualUtility).expect("valid market");
        assert_eq!(first_result, second_result);
        assert_eq!(
            corpus_fingerprint_fnv1a64(&first),
            corpus_fingerprint_fnv1a64(&second)
        );
    }

    #[test]
    fn age_constraints_apply_in_both_directions() {
        let adult_a = person(1, Partition::A, 30, 30, 40, ConnectionGoal::Dating, &[(1, 3)]);
        let too_young = person(2, Partition::B, 29, 18, 40, ConnectionGoal::Dating, &[(1, 3)]);
        assert!(!eligible_pair(&adult_a, &too_young));

        let within_range = person(3, Partition::B, 35, 25, 40, ConnectionGoal::Dating, &[(1, 3)]);
        assert!(eligible_pair(&adult_a, &within_range));
    }

    #[test]
    fn a_block_from_either_side_excludes_the_pair() {
        let mut a = person(1, Partition::A, 30, 18, 60, ConnectionGoal::Dating, &[(1, 3)]);
        let mut b = person(2, Partition::B, 30, 18, 60, ConnectionGoal::Dating, &[(1, 3)]);

        a.blocked_ids.push(2);
        assert!(!eligible_pair(&a, &b));
        a.blocked_ids.clear();

        b.blocked_ids.push(1);
        assert!(!eligible_pair(&a, &b));
    }

    #[test]
    fn mismatched_connection_goals_are_not_recommended() {
        let a = person(1, Partition::A, 30, 18, 60, ConnectionGoal::Dating, &[(1, 3)]);
        let b = person(2, Partition::B, 30, 18, 60, ConnectionGoal::Friendship, &[(1, 3)]);
        assert!(!eligible_pair(&a, &b));
    }

    #[test]
    fn both_rankers_apply_the_same_hard_constraints() {
        let a = person(1, Partition::A, 30, 18, 60, ConnectionGoal::Dating, &[(1, 5)]);
        let b = person(2, Partition::B, 30, 18, 60, ConnectionGoal::Dating, &[(1, 3)]);
        let mut blocked = person(3, Partition::B, 30, 18, 60, ConnectionGoal::Dating, &[(1, 5)]);
        blocked.blocked_ids.push(1);
        let wrong_goal = person(4, Partition::B, 30, 18, 60, ConnectionGoal::Friendship, &[(1, 5)]);
        let people = vec![a, b, blocked, wrong_goal];

        for policy in [RankingPolicy::OneSidedUtility, RankingPolicy::MutualUtility] {
            let rows = recommend(&people, 10, policy).expect("market is structurally valid");
            assert!(rows.iter().all(|row| {
                let viewer = people.iter().find(|person| person.id == row.viewer_id).unwrap();
                let candidate = people.iter().find(|person| person.id == row.candidate_id).unwrap();
                eligible_pair(viewer, candidate)
            }));
            assert!(!rows.iter().any(|row| row.viewer_id == 1 && row.candidate_id == 3));
            assert!(!rows.iter().any(|row| row.viewer_id == 1 && row.candidate_id == 4));
        }
    }

    #[test]
    fn mutual_utility_can_prefer_the_better_two_sided_floor() {
        let viewer = person(
            1,
            Partition::A,
            30,
            18,
            60,
            ConnectionGoal::Dating,
            &[(1, 5), (2, 2)],
        );
        let one_sided_favorite =
            person(2, Partition::B, 30, 18, 60, ConnectionGoal::Dating, &[(1, 1)]);
        let better_floor =
            person(3, Partition::B, 30, 18, 60, ConnectionGoal::Dating, &[(2, 5)]);
        let people = vec![viewer, one_sided_favorite, better_floor];

        let one_sided =
            recommend(&people, 2, RankingPolicy::OneSidedUtility).expect("valid market");
        let mutual = recommend(&people, 2, RankingPolicy::MutualUtility).expect("valid market");

        assert_eq!(one_sided[0].viewer_id, 1);
        assert_eq!(one_sided[0].candidate_id, 2);
        assert_eq!(mutual[0].viewer_id, 1);
        assert_eq!(mutual[0].candidate_id, 3);
    }

    #[test]
    fn malformed_market_fails_closed() {
        let a = person(1, Partition::A, 30, 18, 60, ConnectionGoal::Dating, &[(1, 3)]);
        let duplicate_id = person(1, Partition::B, 30, 18, 60, ConnectionGoal::Dating, &[(1, 3)]);
        assert!(validate_market(&[a, duplicate_id]).is_err());
    }

    #[test]
    fn gini_matches_known_fixture() {
        assert!((gini(&[0, 0, 2, 2]) - 0.5).abs() < 1e-12);
        assert_eq!(gini(&[0, 0, 0]), 0.0);
    }

    #[test]
    fn mutual_shortlist_is_not_mislabeled_as_acceptance() {
        let a = person(1, Partition::A, 30, 18, 60, ConnectionGoal::Dating, &[(1, 3)]);
        let b = person(2, Partition::B, 30, 18, 60, ConnectionGoal::Dating, &[(1, 3)]);
        let people = vec![a, b];
        let rows = recommend(&people, 1, RankingPolicy::MutualUtility).expect("valid market");
        let metrics = evaluate(&people, &rows);

        assert_eq!(metrics.mutual_shortlist_pair_count, 1);
        assert_eq!(metrics.invalid_recommendation_count, 0);
        assert_eq!(metrics.feasible_pair_count, 1);
        assert_eq!(metrics.candidate_coverage, 1.0);
    }
}
