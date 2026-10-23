use std::collections::BTreeMap;
use std::fmt::Write as _;

const PAIR_COUNT: usize = 256;
const PUBLIC_CONTEXT_BUCKETS: usize = 8;
const SESSIONS_PER_WORLD: usize = 4;
const MILLION: u64 = 1_000_000;
const REQUIRED_TASK_RESULT: &str = "ELIGIBLE";
const MANIFEST: &str = include_str!("../channel-manifest.json");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Channel {
    Admission,
    ReasonDetail,
    ResponseSize,
    RoundTrips,
    StatusLookup,
    RetryBehavior,
}

impl Channel {
    const ALL: [Self; 6] = [
        Self::Admission,
        Self::ReasonDetail,
        Self::ResponseSize,
        Self::RoundTrips,
        Self::StatusLookup,
        Self::RetryBehavior,
    ];

    const fn name(self) -> &'static str {
        match self {
            Self::Admission => "admission",
            Self::ReasonDetail => "reason_detail",
            Self::ResponseSize => "response_size",
            Self::RoundTrips => "round_trips",
            Self::StatusLookup => "status_lookup",
            Self::RetryBehavior => "retry_behavior",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scenario {
    NormalizedControl,
    AdmissionOnly,
    ReasonDetailOnly,
    ResponseSizeOnly,
    RoundTripsOnly,
    StatusLookupOnly,
    RetryBehaviorOnly,
    AllChannels,
}

impl Scenario {
    const ALL: [Self; 8] = [
        Self::NormalizedControl,
        Self::AdmissionOnly,
        Self::ReasonDetailOnly,
        Self::ResponseSizeOnly,
        Self::RoundTripsOnly,
        Self::StatusLookupOnly,
        Self::RetryBehaviorOnly,
        Self::AllChannels,
    ];

    const fn name(self) -> &'static str {
        match self {
            Self::NormalizedControl => "normalized_control",
            Self::AdmissionOnly => "admission_only",
            Self::ReasonDetailOnly => "reason_detail_only",
            Self::ResponseSizeOnly => "response_size_only",
            Self::RoundTripsOnly => "round_trips_only",
            Self::StatusLookupOnly => "status_lookup_only",
            Self::RetryBehaviorOnly => "retry_behavior_only",
            Self::AllChannels => "all_channels",
        }
    }

    const fn leaks_channel(self, channel: Channel) -> bool {
        matches!(
            (self, channel),
            (Self::AdmissionOnly, Channel::Admission)
                | (Self::ReasonDetailOnly, Channel::ReasonDetail)
                | (Self::ResponseSizeOnly, Channel::ResponseSize)
                | (Self::RoundTripsOnly, Channel::RoundTrips)
                | (Self::StatusLookupOnly, Channel::StatusLookup)
                | (Self::RetryBehaviorOnly, Channel::RetryBehavior)
                | (Self::AllChannels, _)
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Trace {
    task_result: &'static str,
    public_context_bucket: u8,
    admission_sequence: &'static str,
    reason_detail_sequence: &'static str,
    response_bytes_total: u32,
    round_trips_total: u16,
    status_lookup_count: u8,
    retry_count: u8,
}

fn observe(scenario: Scenario, protected_label: bool, pair_id: usize) -> Trace {
    let correlated = |channel| scenario.leaks_channel(channel) && protected_label;
    let session_count = SESSIONS_PER_WORLD as u8;

    Trace {
        task_result: REQUIRED_TASK_RESULT,
        public_context_bucket: (pair_id % PUBLIC_CONTEXT_BUCKETS) as u8,
        admission_sequence: if correlated(Channel::Admission) {
            "DENY,DENY,DENY,DENY"
        } else {
            "ALLOW,ALLOW,ALLOW,ALLOW"
        },
        reason_detail_sequence: if correlated(Channel::ReasonDetail) {
            "protected-label,generic,generic,generic"
        } else {
            "generic,generic,generic,generic"
        },
        response_bytes_total: if correlated(Channel::ResponseSize) {
            128 * SESSIONS_PER_WORLD as u32
        } else {
            96 * SESSIONS_PER_WORLD as u32
        },
        round_trips_total: if correlated(Channel::RoundTrips) {
            3 * SESSIONS_PER_WORLD as u16
        } else {
            2 * SESSIONS_PER_WORLD as u16
        },
        status_lookup_count: if correlated(Channel::StatusLookup) {
            session_count
        } else {
            0
        },
        retry_count: if correlated(Channel::RetryBehavior) {
            session_count
        } else {
            0
        },
    }
}

#[derive(Clone, Debug)]
struct Example {
    pair_id: usize,
    protected_label: bool,
    trace: Trace,
}

fn examples(scenario: Scenario) -> Vec<Example> {
    let mut result = Vec::with_capacity(PAIR_COUNT * 2);
    for pair_id in 0..PAIR_COUNT {
        for protected_label in [false, true] {
            result.push(Example {
                pair_id,
                protected_label,
                trace: observe(scenario, protected_label, pair_id),
            });
        }
    }
    result
}

fn channel_value(trace: &Trace, channel: Channel) -> String {
    match channel {
        Channel::Admission => trace.admission_sequence.to_owned(),
        Channel::ReasonDetail => trace.reason_detail_sequence.to_owned(),
        Channel::ResponseSize => trace.response_bytes_total.to_string(),
        Channel::RoundTrips => trace.round_trips_total.to_string(),
        Channel::StatusLookup => trace.status_lookup_count.to_string(),
        Channel::RetryBehavior => trace.retry_count.to_string(),
    }
}

fn trace_signature(trace: &Trace) -> String {
    format!(
        "{}|{}|{}|{}|{}|{}|{}|{}",
        trace.task_result,
        trace.public_context_bucket,
        trace.admission_sequence,
        trace.reason_detail_sequence,
        trace.response_bytes_total,
        trace.round_trips_total,
        trace.status_lookup_count,
        trace.retry_count
    )
}

fn paired_mismatches(scenario: Scenario, channel: Option<Channel>) -> usize {
    (0..PAIR_COUNT)
        .filter(|pair_id| {
            let public_world = observe(scenario, false, *pair_id);
            let protected_world = observe(scenario, true, *pair_id);
            match channel {
                Some(field) => {
                    channel_value(&public_world, field) != channel_value(&protected_world, field)
                }
                None => trace_signature(&public_world) != trace_signature(&protected_world),
            }
        })
        .count()
}

fn held_out_balanced_accuracy<F>(data: &[Example], feature: F) -> u64
where
    F: Fn(&Trace) -> String,
{
    // Split whole counterfactual pairs, preventing related worlds from crossing the split.
    let mut training: BTreeMap<String, [usize; 2]> = BTreeMap::new();
    for example in data.iter().filter(|row| row.pair_id % 2 == 0) {
        let counts = training.entry(feature(&example.trace)).or_insert([0, 0]);
        counts[usize::from(example.protected_label)] += 1;
    }

    let mut positive_correct = 0usize;
    let mut positive_total = 0usize;
    let mut negative_correct = 0usize;
    let mut negative_total = 0usize;

    for example in data.iter().filter(|row| row.pair_id % 2 == 1) {
        let counts = training
            .get(&feature(&example.trace))
            .copied()
            .unwrap_or([0, 0]);
        let predicted_label = counts[1] > counts[0];

        if example.protected_label {
            positive_total += 1;
            positive_correct += usize::from(predicted_label);
        } else {
            negative_total += 1;
            negative_correct += usize::from(!predicted_label);
        }
    }

    let sensitivity_ppm = positive_correct as u64 * MILLION / positive_total as u64;
    let specificity_ppm = negative_correct as u64 * MILLION / negative_total as u64;
    (sensitivity_ppm + specificity_ppm) / 2
}

fn task_result_agreement_ppm(data: &[Example]) -> u64 {
    let matching = data
        .iter()
        .filter(|row| row.trace.task_result == REQUIRED_TASK_RESULT)
        .count();
    matching as u64 * MILLION / data.len() as u64
}

fn write_row(
    output: &mut String,
    scenario: Scenario,
    observer_channel: &str,
    data: &[Example],
    mismatch_count: usize,
    accuracy_ppm: u64,
) {
    let mismatch_rate_ppm = mismatch_count as u64 * MILLION / PAIR_COUNT as u64;
    writeln!(
        output,
        "{},{},{},{},{},{},{},{}",
        scenario.name(),
        observer_channel,
        PAIR_COUNT,
        mismatch_count,
        mismatch_rate_ppm,
        accuracy_ppm,
        task_result_agreement_ppm(data),
        false
    )
    .expect("write to string");
}

fn report() -> String {
    let mut output = String::new();
    writeln!(
        output,
        "scenario,observer_channel,counterfactual_pairs,trace_mismatched_pairs,trace_mismatch_rate_ppm,held_out_balanced_accuracy_ppm,task_result_agreement_ppm,wall_clock_timing_measured"
    )
    .expect("write to string");

    for scenario in Scenario::ALL {
        let data = examples(scenario);
        for channel in Channel::ALL {
            let accuracy = held_out_balanced_accuracy(&data, |trace| channel_value(trace, channel));
            write_row(
                &mut output,
                scenario,
                channel.name(),
                &data,
                paired_mismatches(scenario, Some(channel)),
                accuracy,
            );
        }

        let joint_accuracy = held_out_balanced_accuracy(&data, trace_signature);
        write_row(
            &mut output,
            scenario,
            "joint",
            &data,
            paired_mismatches(scenario, None),
            joint_accuracy,
        );
    }

    output
}

fn main() {
    print!("{}", report());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn manifest_names_every_profile_and_observer_channel() {
        for scenario in Scenario::ALL {
            assert!(MANIFEST.contains(scenario.name()));
        }
        for channel in Channel::ALL {
            assert!(MANIFEST.contains(channel.name()));
        }
        assert!(MANIFEST.contains("not measured in this synthetic slice"));
    }

    #[test]
    fn required_task_result_is_identical_in_both_counterfactual_worlds() {
        for scenario in Scenario::ALL {
            for protected_label in [false, true] {
                assert_eq!(
                    observe(scenario, protected_label, 0).task_result,
                    REQUIRED_TASK_RESULT
                );
            }
            assert_eq!(task_result_agreement_ppm(&examples(scenario)), MILLION);
        }
    }

    #[test]
    fn each_profile_changes_only_its_declared_channels() {
        for scenario in Scenario::ALL {
            let public_world = observe(scenario, false, 0);
            let protected_world = observe(scenario, true, 0);
            for channel in Channel::ALL {
                let public_value = channel_value(&public_world, channel);
                let protected_value = channel_value(&protected_world, channel);
                if scenario.leaks_channel(channel) {
                    assert_ne!(
                        public_value,
                        protected_value,
                        "{} / {}",
                        scenario.name(),
                        channel.name()
                    );
                } else {
                    assert_eq!(
                        public_value,
                        protected_value,
                        "{} / {}",
                        scenario.name(),
                        channel.name()
                    );
                }
            }
        }
    }

    #[test]
    fn normalized_control_is_indistinguishable_in_the_declared_trace() {
        assert_eq!(paired_mismatches(Scenario::NormalizedControl, None), 0);
        assert_eq!(
            held_out_balanced_accuracy(&examples(Scenario::NormalizedControl), trace_signature),
            500_000
        );
    }

    #[test]
    fn positive_control_is_detectable_and_channel_ablations_attribute_the_signal() {
        assert_eq!(paired_mismatches(Scenario::AllChannels, None), PAIR_COUNT);
        assert_eq!(
            held_out_balanced_accuracy(&examples(Scenario::AllChannels), trace_signature),
            MILLION
        );

        for (scenario, channel) in [
            (Scenario::AdmissionOnly, Channel::Admission),
            (Scenario::ReasonDetailOnly, Channel::ReasonDetail),
            (Scenario::ResponseSizeOnly, Channel::ResponseSize),
            (Scenario::RoundTripsOnly, Channel::RoundTrips),
            (Scenario::StatusLookupOnly, Channel::StatusLookup),
            (Scenario::RetryBehaviorOnly, Channel::RetryBehavior),
        ] {
            assert_eq!(paired_mismatches(scenario, Some(channel)), PAIR_COUNT);
            assert_eq!(
                held_out_balanced_accuracy(&examples(scenario), |trace| {
                    channel_value(trace, channel)
                }),
                MILLION
            );
        }
    }

    #[test]
    fn report_is_byte_for_byte_deterministic() {
        assert_eq!(report(), report());
    }
}
