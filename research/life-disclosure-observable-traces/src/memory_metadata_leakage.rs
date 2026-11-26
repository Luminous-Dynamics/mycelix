//! Memory-specific longitudinal metadata leakage extension.
//!
//! This composes the generic ObserverView/compare_observer_views interface.
//! The synthetic corpus has 48 sequence seeds (6 memory-state families ×
//! one-shot/repeated-adaptive × single/colluding principals × forbidden/
//! authorised disclosure). Each seed is evaluated with normalized and
//! deliberately leaky observer profiles, and with complete/incomplete trace
//! inventories. Missing observer fields are INDETERMINATE, never PASS.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use crate::{ObserverView, compare_observer_views};

const MEMORY_MANIFEST: &str = include_str!("../memory-sequence-manifest.json");

const REQUIRED_OBSERVER_FIELDS: &[&str] = &[
    "task_result",
    "response_class",
    "response_shape",
    "result_count",
    "ranking_bucket",
    "update_marker",
    "tombstone_marker",
    "cache_membership",
    "query_sequence",
    "principal_count",
    "alice_family_signal",
    "bob_family_signal",
];

const FAMILIES: [MemoryLeakFamily; 6] = [
    MemoryLeakFamily::Existence,
    MemoryLeakFamily::ProtectedValue,
    MemoryLeakFamily::UpdateSupersession,
    MemoryLeakFamily::DeletionTombstone,
    MemoryLeakFamily::QueryMatchRanking,
    MemoryLeakFamily::CacheIndexLifecycle,
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MemoryLeakFamily {
    Existence,
    ProtectedValue,
    UpdateSupersession,
    DeletionTombstone,
    QueryMatchRanking,
    CacheIndexLifecycle,
}

impl MemoryLeakFamily {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Existence => "existence",
            Self::ProtectedValue => "protected_value",
            Self::UpdateSupersession => "update_supersession",
            Self::DeletionTombstone => "deletion_tombstone",
            Self::QueryMatchRanking => "query_match_ranking",
            Self::CacheIndexLifecycle => "cache_index_lifecycle",
        }
    }

    const fn index(self) -> usize {
        match self {
            Self::Existence => 0,
            Self::ProtectedValue => 1,
            Self::UpdateSupersession => 2,
            Self::DeletionTombstone => 3,
            Self::QueryMatchRanking => 4,
            Self::CacheIndexLifecycle => 5,
        }
    }

    const fn signal_field(self) -> &'static str {
        match self {
            Self::Existence => "existence_hint",
            Self::ProtectedValue => "protected_value_hint",
            Self::UpdateSupersession => "update_hint",
            Self::DeletionTombstone => "deletion_hint",
            Self::QueryMatchRanking => "match_ranking_hint",
            Self::CacheIndexLifecycle => "cache_index_hint",
        }
    }

    const fn signal_value(self, protected_world: bool) -> &'static str {
        match (self, protected_world) {
            (Self::Existence, false) => "ABSENT",
            (Self::Existence, true) => "PRESENT",
            (Self::ProtectedValue, false) => "VALUE_BETA",
            (Self::ProtectedValue, true) => "VALUE_ALPHA",
            (Self::UpdateSupersession, false) => "REVISION_1",
            (Self::UpdateSupersession, true) => "REVISION_2",
            (Self::DeletionTombstone, false) => "NO_TOMBSTONE",
            (Self::DeletionTombstone, true) => "TOMBSTONE",
            (Self::QueryMatchRanking, false) => "NO_MATCH",
            (Self::QueryMatchRanking, true) => "RANK_1",
            (Self::CacheIndexLifecycle, false) => "INDEX_MISS",
            (Self::CacheIndexLifecycle, true) => "INDEX_HIT",
        }
    }

    const fn signal_recipient(self) -> &'static str {
        if self.index().is_multiple_of(2) {
            "alice"
        } else {
            "bob"
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QueryMode {
    OneShot,
    RepeatedAdaptive,
}

impl QueryMode {
    const fn name(self) -> &'static str {
        match self {
            Self::OneShot => "one_shot",
            Self::RepeatedAdaptive => "repeated_adaptive",
        }
    }

    const fn session_count(self) -> usize {
        match self {
            Self::OneShot => 1,
            Self::RepeatedAdaptive => 4,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PrincipalMode {
    Single,
    Colluding,
}

impl PrincipalMode {
    const fn name(self) -> &'static str {
        match self {
            Self::Single => "single_principal",
            Self::Colluding => "colluding_principals",
        }
    }

    const fn principals(self) -> &'static [&'static str] {
        match self {
            Self::Single => &["alice"],
            Self::Colluding => &["alice", "bob"],
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DisclosureMode {
    Forbidden,
    Authorised,
}

impl DisclosureMode {
    const fn name(self) -> &'static str {
        match self {
            Self::Forbidden => "forbidden",
            Self::Authorised => "authorised",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ObserverProfile {
    Normalized,
    FamilyLeakControl,
}

impl ObserverProfile {
    const fn name(self) -> &'static str {
        match self {
            Self::Normalized => "normalized_control",
            Self::FamilyLeakControl => "family_leak_positive_control",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraceCoverage {
    Complete,
    Incomplete,
}

impl TraceCoverage {
    const fn name(self) -> &'static str {
        match self {
            Self::Complete => "complete",
            Self::Incomplete => "incomplete",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LeakageDisposition {
    Pass,
    LeakDetected,
    Indeterminate,
}

impl LeakageDisposition {
    const fn name(self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::LeakDetected => "LEAK_DETECTED",
            Self::Indeterminate => "INDETERMINATE",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SequenceSeed {
    pub sequence_id: u8,
    pub family: MemoryLeakFamily,
    pub query_mode: QueryMode,
    pub principal_mode: PrincipalMode,
    pub disclosure_mode: DisclosureMode,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SequenceEvaluation {
    pub sequence: SequenceSeed,
    pub observer_profile: ObserverProfile,
    pub trace_coverage: TraceCoverage,
    pub disposition: LeakageDisposition,
    pub mismatched_fields: Vec<String>,
    pub missing_fields: Vec<String>,
    pub attacker_balanced_accuracy_ppm: Option<u64>,
    pub authorised_sensitivity_detected: bool,
}

fn sequence_seeds() -> Vec<SequenceSeed> {
    let mut result = Vec::with_capacity(48);
    let mut sequence_id = 0u8;
    for family in FAMILIES {
        for query_mode in [QueryMode::OneShot, QueryMode::RepeatedAdaptive] {
            for principal_mode in [PrincipalMode::Single, PrincipalMode::Colluding] {
                for disclosure_mode in [DisclosureMode::Forbidden, DisclosureMode::Authorised] {
                    result.push(SequenceSeed {
                        sequence_id,
                        family,
                        query_mode,
                        principal_mode,
                        disclosure_mode,
                    });
                    sequence_id += 1;
                }
            }
        }
    }
    result
}

fn simulate_observer_view(
    sequence: SequenceSeed,
    protected_world: bool,
    profile: ObserverProfile,
    coverage: TraceCoverage,
) -> ObserverView {
    let principals = sequence.principal_mode.principals();
    let signal_recipient = sequence.family.signal_recipient();
    let mut principal_signals = BTreeMap::<&str, Vec<&str>>::new();
    for principal in principals {
        principal_signals.insert(
            principal,
            Vec::with_capacity(sequence.query_mode.session_count()),
        );
    }

    let mut query_sequence = Vec::with_capacity(sequence.query_mode.session_count());
    let mut prior_visible_signal = false;

    for session in 0..sequence.query_mode.session_count() {
        // Adaptive follow-up is derived only from the previous observer-visible
        // signal, never from the hidden protected-world label.
        let query = if session == 0 {
            "initial_lookup"
        } else if sequence.query_mode == QueryMode::RepeatedAdaptive && prior_visible_signal {
            "verify_prior_candidate"
        } else {
            "generic_status_poll"
        };
        query_sequence.push(query);

        let mut any_visible_signal = false;
        for principal in principals {
            let signal = if sequence.disclosure_mode == DisclosureMode::Forbidden
                && profile == ObserverProfile::FamilyLeakControl
                && *principal == signal_recipient
            {
                sequence.family.signal_value(protected_world)
            } else {
                "OPAQUE"
            };
            // The attacker has a preregistered target value and adapts
            // only when that value appears in its visible transcript.
            // Merely seeing any response (including ABSENT) is not enough.
            if signal == sequence.family.signal_value(true) {
                any_visible_signal = true;
            }
            principal_signals
                .get_mut(principal)
                .expect("principal signal slot exists")
                .push(signal);
        }
        prior_visible_signal = any_visible_signal;
    }

    let join_signals = |principal: &str| {
        principal_signals
            .get(principal)
            .map(|signals| signals.join("|"))
            .unwrap_or_else(|| "NOT_PRESENT".to_owned())
    };

    let mut view = ObserverView::from([
        ("task_result".into(), "ELIGIBLE".into()),
        (
            "response_class".into(),
            if sequence.disclosure_mode == DisclosureMode::Authorised {
                "AUTHORISED".into()
            } else {
                "WITHHELD".into()
            },
        ),
        ("response_shape".into(), "FIXED_GENERIC".into()),
        ("result_count".into(), "0".into()),
        ("ranking_bucket".into(), "OPAQUE".into()),
        ("update_marker".into(), "OPAQUE".into()),
        ("tombstone_marker".into(), "OPAQUE".into()),
        ("cache_membership".into(), "OPAQUE".into()),
        ("query_sequence".into(), query_sequence.join("|")),
        ("principal_count".into(), principals.len().to_string()),
        ("alice_family_signal".into(), join_signals("alice")),
        ("bob_family_signal".into(), join_signals("bob")),
        (sequence.family.signal_field().into(), "OPAQUE".into()),
        (
            "authorized_disclosure".into(),
            if sequence.disclosure_mode == DisclosureMode::Authorised {
                sequence.family.signal_value(protected_world).into()
            } else {
                "NONE".into()
            },
        ),
    ]);

    // A family leak is emitted through the declared observer-visible channel
    // by one principal. Collusion unions both principals' actual visible traces.
    if sequence.disclosure_mode == DisclosureMode::Forbidden
        && profile == ObserverProfile::FamilyLeakControl
        && principals.contains(&signal_recipient)
    {
        view.insert(
            sequence.family.signal_field().into(),
            sequence.family.signal_value(protected_world).into(),
        );
    }

    if coverage == TraceCoverage::Incomplete {
        // Simulate a collector that failed to record one required observer
        // field in both worlds. Absence is an evidence gap, not equality.
        view.remove("cache_membership");
    }

    view
}

fn family_feature(sequence: SequenceSeed, view: &ObserverView) -> String {
    let field = sequence.family.signal_field();
    view.get(field).cloned().unwrap_or_else(|| "MISSING".into())
}

fn attacker_balanced_accuracy(
    family: MemoryLeakFamily,
    principal_mode: PrincipalMode,
    profile: ObserverProfile,
    coverage: TraceCoverage,
) -> u64 {
    let seeds = sequence_seeds();
    let mut training = BTreeMap::<String, [usize; 2]>::new();

    for seed in seeds.iter().filter(|seed| {
        seed.family == family
            && seed.principal_mode == principal_mode
            && seed.disclosure_mode == DisclosureMode::Forbidden
            && seed.query_mode == QueryMode::OneShot
    }) {
        for protected_world in [false, true] {
            let view = simulate_observer_view(*seed, protected_world, profile, coverage);
            let counts = training
                .entry(family_feature(*seed, &view))
                .or_insert([0, 0]);
            counts[usize::from(protected_world)] += 1;
        }
    }

    let mut positive_correct = 0u64;
    let mut positive_total = 0u64;
    let mut negative_correct = 0u64;
    let mut negative_total = 0u64;

    // Test on repeated/adaptive sequences after training only on one-shot
    // sequences. This is a sequence-generalisation test, not random row split.
    for seed in seeds.iter().filter(|seed| {
        seed.family == family
            && seed.principal_mode == principal_mode
            && seed.disclosure_mode == DisclosureMode::Forbidden
            && seed.query_mode == QueryMode::RepeatedAdaptive
    }) {
        for protected_world in [false, true] {
            let view = simulate_observer_view(*seed, protected_world, profile, coverage);
            let counts = training
                .get(&family_feature(*seed, &view))
                .copied()
                .unwrap_or([0, 0]);
            let predicted = counts[1] > counts[0];
            if protected_world {
                positive_total += 1;
                positive_correct += u64::from(predicted);
            } else {
                negative_total += 1;
                negative_correct += u64::from(!predicted);
            }
        }
    }

    if positive_total == 0 || negative_total == 0 {
        return 500_000;
    }
    let sensitivity = positive_correct * 1_000_000 / positive_total;
    let specificity = negative_correct * 1_000_000 / negative_total;
    (sensitivity + specificity) / 2
}

pub fn evaluate_sequence(
    sequence: SequenceSeed,
    observer_profile: ObserverProfile,
    trace_coverage: TraceCoverage,
) -> SequenceEvaluation {
    let public_view = simulate_observer_view(sequence, false, observer_profile, trace_coverage);
    let protected_view = simulate_observer_view(sequence, true, observer_profile, trace_coverage);
    // The family-specific signal channel is part of this experiment's
    // declared observer view. Add it explicitly to the generic required-field
    // list so the comparator reports which memory metadata field diverged.
    let mut required_fields = REQUIRED_OBSERVER_FIELDS.to_vec();
    required_fields.push(sequence.family.signal_field());
    let comparison =
        compare_observer_views(&public_view, &protected_view, &required_fields);

    let authorised_sensitivity_detected = sequence.disclosure_mode == DisclosureMode::Authorised
        && public_view.get("authorized_disclosure") != protected_view.get("authorized_disclosure");

    let disposition = if !comparison.is_complete() {
        LeakageDisposition::Indeterminate
    } else if comparison.mismatched_fields.is_empty() {
        LeakageDisposition::Pass
    } else {
        LeakageDisposition::LeakDetected
    };

    let attacker_accuracy = if sequence.disclosure_mode == DisclosureMode::Forbidden {
        Some(attacker_balanced_accuracy(
            sequence.family,
            sequence.principal_mode,
            observer_profile,
            trace_coverage,
        ))
    } else {
        None
    };

    SequenceEvaluation {
        sequence,
        observer_profile,
        trace_coverage,
        disposition,
        mismatched_fields: comparison.mismatched_fields,
        missing_fields: comparison.missing_fields,
        attacker_balanced_accuracy_ppm: attacker_accuracy,
        authorised_sensitivity_detected,
    }
}

pub fn evaluate_panel() -> Vec<SequenceEvaluation> {
    let mut results = Vec::with_capacity(192);
    for seed in sequence_seeds() {
        for profile in [
            ObserverProfile::Normalized,
            ObserverProfile::FamilyLeakControl,
        ] {
            for coverage in [TraceCoverage::Complete, TraceCoverage::Incomplete] {
                results.push(evaluate_sequence(seed, profile, coverage));
            }
        }
    }
    results
}

pub fn report_csv() -> String {
    let mut output = String::new();
    writeln!(
        output,
        "sequence_id,family,query_mode,principal_mode,disclosure_mode,observer_profile,trace_coverage,disposition,mismatch_fields,missing_fields,attacker_balanced_accuracy_ppm,authorised_sensitivity_detected"
    )
    .expect("write header");

    for result in evaluate_panel() {
        writeln!(
            output,
            "{},{},{},{},{},{},{},{},{},{},{},{}",
            result.sequence.sequence_id,
            result.sequence.family.name(),
            result.sequence.query_mode.name(),
            result.sequence.principal_mode.name(),
            result.sequence.disclosure_mode.name(),
            result.observer_profile.name(),
            result.trace_coverage.name(),
            result.disposition.name(),
            result.mismatched_fields.join("|"),
            result.missing_fields.join("|"),
            result
                .attacker_balanced_accuracy_ppm
                .map(|value| value.to_string())
                .unwrap_or_else(|| "NA".into()),
            result.authorised_sensitivity_detected
        )
        .expect("write result row");
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    fn select(
        family: MemoryLeakFamily,
        query_mode: QueryMode,
        principal_mode: PrincipalMode,
        disclosure_mode: DisclosureMode,
    ) -> SequenceSeed {
        sequence_seeds()
            .into_iter()
            .find(|seed| {
                seed.family == family
                    && seed.query_mode == query_mode
                    && seed.principal_mode == principal_mode
                    && seed.disclosure_mode == disclosure_mode
            })
            .expect("seed exists")
    }

    #[test]
    fn panel_has_48_seed_sequences_and_192_qualification_rows() {
        assert_eq!(sequence_seeds().len(), 48);
        assert_eq!(evaluate_panel().len(), 192);
    }

    #[test]
    fn normalized_forbidden_controls_have_no_observed_difference() {
        for seed in sequence_seeds()
            .into_iter()
            .filter(|seed| seed.disclosure_mode == DisclosureMode::Forbidden)
        {
            let result =
                evaluate_sequence(seed, ObserverProfile::Normalized, TraceCoverage::Complete);
            assert_eq!(result.disposition, LeakageDisposition::Pass, "{seed:?}");
            assert!(result.mismatched_fields.is_empty());
            assert!(!result.authorised_sensitivity_detected);
        }
    }

    #[test]
    fn family_specific_positive_controls_detect_all_six_metadata_channels_under_collusion() {
        for family in FAMILIES {
            let seed = select(
                family,
                QueryMode::RepeatedAdaptive,
                PrincipalMode::Colluding,
                DisclosureMode::Forbidden,
            );
            let result = evaluate_sequence(
                seed,
                ObserverProfile::FamilyLeakControl,
                TraceCoverage::Complete,
            );
            assert_eq!(
                result.disposition,
                LeakageDisposition::LeakDetected,
                "{family:?}"
            );
            assert!(
                result
                    .mismatched_fields
                    .contains(&family.signal_field().to_owned())
            );
            assert_eq!(result.attacker_balanced_accuracy_ppm, Some(1_000_000));
        }
    }

    #[test]
    fn collusion_combines_a_bob_only_signal_that_single_principal_cannot_observe() {
        let single = select(
            MemoryLeakFamily::ProtectedValue,
            QueryMode::RepeatedAdaptive,
            PrincipalMode::Single,
            DisclosureMode::Forbidden,
        );
        let colluding = select(
            MemoryLeakFamily::ProtectedValue,
            QueryMode::RepeatedAdaptive,
            PrincipalMode::Colluding,
            DisclosureMode::Forbidden,
        );
        let single_result = evaluate_sequence(
            single,
            ObserverProfile::FamilyLeakControl,
            TraceCoverage::Complete,
        );
        let colluding_result = evaluate_sequence(
            colluding,
            ObserverProfile::FamilyLeakControl,
            TraceCoverage::Complete,
        );
        assert_eq!(single_result.disposition, LeakageDisposition::Pass);
        assert_eq!(single_result.attacker_balanced_accuracy_ppm, Some(500_000));
        assert_eq!(
            colluding_result.disposition,
            LeakageDisposition::LeakDetected
        );
        assert_eq!(
            colluding_result.attacker_balanced_accuracy_ppm,
            Some(1_000_000)
        );
    }

    #[test]
    fn authorised_disclosure_has_positive_sensitivity_without_forbidden_channel_changes() {
        let seed = select(
            MemoryLeakFamily::ProtectedValue,
            QueryMode::RepeatedAdaptive,
            PrincipalMode::Single,
            DisclosureMode::Authorised,
        );
        for profile in [
            ObserverProfile::Normalized,
            ObserverProfile::FamilyLeakControl,
        ] {
            let result = evaluate_sequence(seed, profile, TraceCoverage::Complete);
            assert_eq!(result.disposition, LeakageDisposition::Pass);
            assert!(result.mismatched_fields.is_empty());
            assert!(result.authorised_sensitivity_detected);
        }
    }

    #[test]
    fn incomplete_observer_inventory_is_indeterminate_not_a_pass() {
        let seed = select(
            MemoryLeakFamily::Existence,
            QueryMode::OneShot,
            PrincipalMode::Single,
            DisclosureMode::Forbidden,
        );
        let result =
            evaluate_sequence(seed, ObserverProfile::Normalized, TraceCoverage::Incomplete);
        assert_eq!(result.disposition, LeakageDisposition::Indeterminate);
        assert_eq!(result.missing_fields, vec!["cache_membership"]);
    }

    #[test]
    fn adaptive_query_sequence_is_derived_from_visible_history() {
        let seed = select(
            MemoryLeakFamily::Existence,
            QueryMode::RepeatedAdaptive,
            PrincipalMode::Single,
            DisclosureMode::Forbidden,
        );
        let normalized_public = simulate_observer_view(
            seed,
            false,
            ObserverProfile::Normalized,
            TraceCoverage::Complete,
        );
        let normalized_protected = simulate_observer_view(
            seed,
            true,
            ObserverProfile::Normalized,
            TraceCoverage::Complete,
        );
        assert_eq!(
            normalized_public.get("query_sequence"),
            normalized_protected.get("query_sequence")
        );

        let leaked_public = simulate_observer_view(
            seed,
            false,
            ObserverProfile::FamilyLeakControl,
            TraceCoverage::Complete,
        );
        let leaked_protected = simulate_observer_view(
            seed,
            true,
            ObserverProfile::FamilyLeakControl,
            TraceCoverage::Complete,
        );
        assert_eq!(
            leaked_public.get("query_sequence").unwrap(),
            "initial_lookup|generic_status_poll|generic_status_poll|generic_status_poll"
        );
        assert_eq!(
            leaked_protected.get("query_sequence").unwrap(),
            "initial_lookup|verify_prior_candidate|verify_prior_candidate|verify_prior_candidate"
        );
        assert_ne!(
            leaked_public.get("query_sequence"),
            leaked_protected.get("query_sequence")
        );
    }

    #[test]
    fn every_memory_family_has_a_leak_control_and_forbidden_and_authorised_modes() {
        for family in FAMILIES {
            for principal_mode in [PrincipalMode::Single, PrincipalMode::Colluding] {
                let seed = select(
                    family,
                    QueryMode::RepeatedAdaptive,
                    principal_mode,
                    DisclosureMode::Forbidden,
                );
                let result = evaluate_sequence(
                    seed,
                    ObserverProfile::FamilyLeakControl,
                    TraceCoverage::Complete,
                );
                let should_reach_observer = principal_mode == PrincipalMode::Colluding
                    || family.signal_recipient() == "alice";
                assert_eq!(
                    result.disposition == LeakageDisposition::LeakDetected,
                    should_reach_observer,
                    "{family:?} / {principal_mode:?}"
                );
            }

            let authorised = select(
                family,
                QueryMode::RepeatedAdaptive,
                PrincipalMode::Single,
                DisclosureMode::Authorised,
            );
            let result = evaluate_sequence(
                authorised,
                ObserverProfile::Normalized,
                TraceCoverage::Complete,
            );
            assert_eq!(result.disposition, LeakageDisposition::Pass);
            assert!(result.authorised_sensitivity_detected);
        }
    }

    #[test]
    fn unknown_trace_coverage_never_becomes_a_privacy_pass_even_for_leaky_worlds() {
        for family in FAMILIES {
            let seed = select(
                family,
                QueryMode::RepeatedAdaptive,
                PrincipalMode::Colluding,
                DisclosureMode::Forbidden,
            );
            let result = evaluate_sequence(
                seed,
                ObserverProfile::FamilyLeakControl,
                TraceCoverage::Incomplete,
            );
            assert_eq!(result.disposition, LeakageDisposition::Indeterminate);
            assert_eq!(result.missing_fields, vec!["cache_membership"]);
        }
    }

    #[test]
    fn evaluator_only_state_and_labels_are_not_part_of_the_observer_view() {
        let seed = select(
            MemoryLeakFamily::Existence,
            QueryMode::RepeatedAdaptive,
            PrincipalMode::Colluding,
            DisclosureMode::Forbidden,
        );
        for protected_world in [false, true] {
            let view = simulate_observer_view(
                seed,
                protected_world,
                ObserverProfile::Normalized,
                TraceCoverage::Complete,
            );
            assert!(!view.contains_key("protected_world"));
            assert!(!view.contains_key("protected_label"));
            assert!(!view.contains_key("evaluator_truth"));
            assert!(view.contains_key("task_result"));
        }
    }

    #[test]
    fn manifest_freezes_the_exact_48_sequence_cross_product_and_claim_ceiling() {
        assert!(
            MEMORY_MANIFEST
                .contains("\"schema_version\": \"mycelix-memory-longitudinal-leakage-v0.1\"")
        );
        assert!(MEMORY_MANIFEST.contains("\"seed_count\": 48"));
        assert!(MEMORY_MANIFEST.contains("\"evaluation_rows\": 192"));
        assert!(MEMORY_MANIFEST.contains("\"sequence_id\": 47"));
        for family in FAMILIES {
            assert!(MEMORY_MANIFEST.contains(family.name()));
        }
        for value in [
            "one_shot",
            "repeated_adaptive",
            "single_principal",
            "colluding_principals",
            "forbidden",
            "authorised",
            "normalized_control",
            "family_leak_positive_control",
            "INDETERMINATE",
            "not a production privacy guarantee",
        ] {
            assert!(MEMORY_MANIFEST.contains(value), "manifest missing {value}");
        }
    }

    #[test]
    fn report_has_one_header_and_exactly_192_rows() {
        let report = report_csv();
        assert_eq!(report.lines().count(), 193);
        assert!(report.lines().any(|line| line.contains(",LEAK_DETECTED,")));
        assert!(report.lines().any(|line| line.contains(",INDETERMINATE,")));
        assert!(report.lines().any(|line| line.contains(",PASS,")));
    }

    #[test]
    fn report_is_byte_for_byte_deterministic() {
        assert_eq!(report_csv(), report_csv());
    }
}
